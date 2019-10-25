/** $glic$
 * Copyright (C) 2017 by Google Inc.
 * Copyright (C) 2012-2015 by Massachusetts Institute of Technology
 * Copyright (C) 2010-2013 by The Board of Trustees of Stanford University
 * Copyright (C) 2011 Google Inc.
 *
 * This file is part of zsim.
 *
 * zsim is free software; you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, version 2.
 *
 * If you use this software in your research, we request that you reference
 * the zsim paper ("ZSim: Fast and Accurate Microarchitectural Simulation of
 * Thousand-Core Systems", Sanchez and Kozyrakis, ISCA-40, June 2013) as the
 * source of the simulator in any publications that use this software, and that
 * you send us a citation of your work.
 *
 * zsim is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "init.h"
#include <list>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <sys/time.h>
#include <vector>
#include "cache.h"
#include "cache_arrays.h"
#include "config.h"
#include "constants.h"
#include "contention_sim.h"
#include "core.h"
#include "detailed_mem.h"
#include "detailed_mem_params.h"
#include "ddr_mem.h"
#include "debug_zsim.h"
#include "dramsim_mem_ctrl.h"
#include "event_queue.h"
#include "filter_cache.h"
#include "galloc.h"
#include "hash.h"
#include "ideal_arrays.h"
#include "locks.h"
#include "log.h"
#include "mem_ctrls.h"
#include "network.h"
#include "next_line_prefetcher.h"
#include "null_core.h"
#include "ooo_core.h"
#include "part_repl_policies.h"
#include "prefetcher.h"
#include "ghb_prefetcher.h"
#include "profile_stats.h"
#include "repl_policies.h"
#include "simple_core.h"
#include "stats.h"
#include "stats_filter.h"
#include "str.h"
#include "table_prefetcher.h"
#include "timing_cache.h"
#include "timing_core.h"
#include "timing_event.h"
#include "trace_driver.h"
#include "tracing_cache.h"
#include "weave_md1_mem.h" //validation, could be taken out...
#ifdef ZSIM_USE_YT
#include "yt_tracer.h"
#endif  // ZSIM_USE_YT
#include "zsim.h"

using namespace std;

extern void EndOfPhaseActions(); //in zsim.cpp

/* zsim should be initialized in a deterministic and logical order, to avoid re-reading config vars
 * all over the place and give a predictable global state to constructors. Ideally, this should just
 * follow the layout of zinfo, top-down.
 */

BaseCache* BuildCacheBank(Config& config, const string& prefix, g_string& name, uint32_t bankSize, bool isTerminal, uint32_t domain) {
    string type = config.get<const char*>(prefix + "type", "Simple");
    // Shortcut for TraceDriven type
    if (type == "TraceDriven") {
        assert(zinfo->traceDriven);
        assert(isTerminal);
        return new TraceDriverProxyCache(name);
    }

    uint32_t lineSize = zinfo->lineSize;
    assert(lineSize > 0); //avoid config deps
    if (bankSize % lineSize != 0) panic("%s: Bank size must be a multiple of line size", name.c_str());

    uint32_t numLines = bankSize/lineSize;

    //Array
    uint32_t numHashes = 1;
    uint32_t ways = config.get<uint32_t>(prefix + "array.ways", 4);
    string arrayType = config.get<const char*>(prefix + "array.type", "SetAssoc");
    uint32_t candidates = (arrayType == "Z")? config.get<uint32_t>(prefix + "array.candidates", 16) : ways;

    //Need to know number of hash functions before instantiating array
    if (arrayType == "SetAssoc") {
        numHashes = 1;
    } else if (arrayType == "Z") {
        numHashes = ways;
        assert(ways > 1);
    } else if (arrayType == "IdealLRU" || arrayType == "IdealLRUPart") {
        ways = numLines;
        numHashes = 0;
    } else {
        panic("%s: Invalid array type %s", name.c_str(), arrayType.c_str());
    }

    // Power of two sets check; also compute setBits, will be useful later
    uint32_t numSets = numLines/ways;
    uint32_t setBits = 31 - __builtin_clz(numSets);
    if ((1u << setBits) != numSets) panic("%s: Number of sets must be a power of two (you specified %d sets)", name.c_str(), numSets);

    //Hash function
    HashFamily* hf = nullptr;
    string hashType = config.get<const char*>(prefix + "array.hash", (arrayType == "Z")? "H3" : "None"); //zcaches must be hashed by default
    if (numHashes) {
        if (hashType == "None") {
            if (arrayType == "Z") panic("ZCaches must be hashed!"); //double check for stupid user
            assert(numHashes == 1);
            hf = new IdHashFamily;
        } else if (hashType == "H3") {
            //STL hash function
            size_t seed = _Fnv_hash_bytes(prefix.c_str(), prefix.size()+1, 0xB4AC5B);
            //info("%s -> %lx", prefix.c_str(), seed);
            hf = new H3HashFamily(numHashes, setBits, 0xCAC7EAFFA1 + seed /*make randSeed depend on prefix*/);
        } else if (hashType == "SHA1") {
            hf = new SHA1HashFamily(numHashes);
        } else {
            panic("%s: Invalid value %s on array.hash", name.c_str(), hashType.c_str());
        }
    }

    //Replacement policy
    string replType = config.get<const char*>(prefix + "repl.type", (arrayType == "IdealLRUPart")? "IdealLRUPart" : "LRU");
    ReplPolicy* rp = nullptr;

    if (replType == "LRU" || replType == "LRUNoSh") {
        bool sharersAware = (replType == "LRU") && !isTerminal;
        if (sharersAware) {
            rp = new LRUReplPolicy<true>(numLines);
        } else {
            rp = new LRUReplPolicy<false>(numLines);
        }
    } else if (replType == "LFU") {
        rp = new LFUReplPolicy(numLines);
    } else if (replType == "LRUProfViol") {
        ProfViolReplPolicy< LRUReplPolicy<true> >* pvrp = new ProfViolReplPolicy< LRUReplPolicy<true> >(numLines);
        pvrp->init(numLines);
        rp = pvrp;
    } else if (replType == "TreeLRU") {
        rp = new TreeLRUReplPolicy(numLines, candidates);
    } else if (replType == "NRU") {
        rp = new NRUReplPolicy(numLines, candidates);
    } else if (replType == "Rand") {
        rp = new RandReplPolicy(candidates);
    } else if (replType == "WayPart" || replType == "Vantage" || replType == "IdealLRUPart") {
        if (replType == "WayPart" && arrayType != "SetAssoc") panic("WayPart replacement requires SetAssoc array");

        //Partition mapper
        // TODO: One partition mapper per cache (not bank).
        string partMapper = config.get<const char*>(prefix + "repl.partMapper", "Core");
        PartMapper* pm = nullptr;
        if (partMapper == "Core") {
            pm = new CorePartMapper(zinfo->numCores); //NOTE: If the cache is not fully shared, trhis will be inefficient...
        } else if (partMapper == "InstrData") {
            pm = new InstrDataPartMapper();
        } else if (partMapper == "InstrDataCore") {
            pm = new InstrDataCorePartMapper(zinfo->numCores);
        } else if (partMapper == "Process") {
            pm = new ProcessPartMapper(zinfo->numProcs);
        } else if (partMapper == "InstrDataProcess") {
            pm = new InstrDataProcessPartMapper(zinfo->numProcs);
        } else if (partMapper == "ProcessGroup") {
            pm = new ProcessGroupPartMapper();
        } else {
            panic("Invalid repl.partMapper %s on %s", partMapper.c_str(), name.c_str());
        }

        // Partition monitor
        uint32_t umonLines = config.get<uint32_t>(prefix + "repl.umonLines", 256);
        uint32_t umonWays = config.get<uint32_t>(prefix + "repl.umonWays", ways);
        uint32_t buckets;
        if (replType == "WayPart") {
            buckets = ways; //not an option with WayPart
        } else { //Vantage or Ideal
            buckets = config.get<uint32_t>(prefix + "repl.buckets", 256);
        }

        PartitionMonitor* mon = new UMonMonitor(numLines, umonLines, umonWays, pm->getNumPartitions(), buckets);

        //Finally, instantiate the repl policy
        PartReplPolicy* prp;
        double allocPortion = 1.0;
        if (replType == "WayPart") {
            //if set, drives partitioner but doesn't actually do partitioning
            bool testMode = config.get<bool>(prefix + "repl.testMode", false);
            prp = new WayPartReplPolicy(mon, pm, numLines, ways, testMode);
        } else if (replType == "IdealLRUPart") {
            prp = new IdealLRUPartReplPolicy(mon, pm, numLines, buckets);
        } else { //Vantage
            uint32_t assoc = (arrayType == "Z")? candidates : ways;
            allocPortion = .85;
            bool smoothTransients = config.get<bool>(prefix + "repl.smoothTransients", false);
            prp = new VantageReplPolicy(mon, pm, numLines, assoc, (uint32_t)(allocPortion * 100), 10, 50, buckets, smoothTransients);
        }
        rp = prp;

        // Partitioner
        // TODO: Depending on partitioner type, we want one per bank or one per cache.
        Partitioner* p = new LookaheadPartitioner(prp, pm->getNumPartitions(), buckets, 1, allocPortion);

        //Schedule its tick
        uint32_t interval = config.get<uint32_t>(prefix + "repl.interval", 5000); //phases
        zinfo->eventQueue->insert(new Partitioner::PartitionEvent(p, interval));
    } else {
        panic("%s: Invalid replacement type %s", name.c_str(), replType.c_str());
    }
    assert(rp);


    //Alright, build the array
    CacheArray* array = nullptr;
    if (arrayType == "SetAssoc") {
        array = new SetAssocArray(numLines, ways, rp, hf);
    } else if (arrayType == "Z") {
        array = new ZArray(numLines, ways, candidates, rp, hf);
    } else if (arrayType == "IdealLRU") {
        assert(replType == "LRU");
        assert(!hf);
        IdealLRUArray* ila = new IdealLRUArray(numLines);
        rp = ila->getRP();
        array = ila;
    } else if (arrayType == "IdealLRUPart") {
        assert(!hf);
        IdealLRUPartReplPolicy* irp = dynamic_cast<IdealLRUPartReplPolicy*>(rp);
        if (!irp) panic("IdealLRUPart array needs IdealLRUPart repl policy!");
        array = new IdealLRUPartArray(numLines, irp);
    } else {
        panic("This should not happen, we already checked for it!"); //unless someone changed arrayStr...
    }

    //Latency
    uint32_t latency = config.get<uint32_t>(prefix + "latency", 10);
    uint32_t accLat = (isTerminal)? 0 : latency; //terminal caches has no access latency b/c it is assumed accLat is hidden by the pipeline
    uint32_t invLat = latency;

    // Inclusion?
    bool nonInclusiveHack = config.get<bool>(prefix + "nonInclusiveHack", false);
    if (nonInclusiveHack) assert(type == "Simple" && !isTerminal);

    // Finally, build the cache
    Cache* cache;
    CC* cc;
    if (isTerminal) {
        cc = new MESITerminalCC(numLines, name);
    } else {
        cc = new MESICC(numLines, nonInclusiveHack, name);
    }
    rp->setCC(cc);
    if (!isTerminal) {
        if (type == "Simple") {
            cache = new Cache(numLines, cc, array, rp, accLat, invLat, name);
        } else if (type == "Timing") {
            uint32_t mshrs = config.get<uint32_t>(prefix + "mshrs", 16);
            uint32_t tagLat = config.get<uint32_t>(prefix + "tagLat", 5);
            uint32_t timingCandidates = config.get<uint32_t>(prefix + "timingCandidates", candidates);
            cache = new TimingCache(numLines, cc, array, rp, accLat, invLat, mshrs, tagLat, ways, timingCandidates, domain, name);
        } else if (type == "Tracing") {
            g_string traceFile = config.get<const char*>(prefix + "traceFile","");
            if (traceFile.empty()) traceFile = g_string(zinfo->outputDir) + "/" + name + ".trace";
            cache = new TracingCache(numLines, cc, array, rp, accLat, invLat, traceFile, name);
        } else {
            panic("Invalid cache type %s", type.c_str());
        }
    } else {
        //Filter cache optimization
        if (type != "Simple") panic("Terminal cache %s can only have type == Simple", name.c_str());
        if (arrayType != "SetAssoc" || hashType != "None" || replType != "LRU") panic("Invalid FilterCache config %s", name.c_str());
        cache = new FilterCache(numSets, numLines, cc, array, rp, accLat, invLat, name);
    }

#if 0
    info("Built L%d bank, %d bytes, %d lines, %d ways (%d candidates if array is Z), %s array, %s hash, %s replacement, accLat %d, invLat %d name %s",
            level, bankSize, numLines, ways, candidates, arrayType.c_str(), hashType.c_str(), replType.c_str(), accLat, invLat, name.c_str());
#endif

    return cache;
}

// NOTE: frequency is SYSTEM frequency; mem freq specified in tech
DDRMemory* BuildDDRMemory(Config& config, uint32_t lineSize, uint32_t frequency, uint32_t domain, g_string name, const string& prefix) {
    uint32_t ranksPerChannel = config.get<uint32_t>(prefix + "ranksPerChannel", 4);
    uint32_t banksPerRank = config.get<uint32_t>(prefix + "banksPerRank", 8);  // DDR3 std is 8
    uint32_t pageSize = config.get<uint32_t>(prefix + "pageSize", 8*1024);  // 1Kb cols, x4 devices
    const char* tech = config.get<const char*>(prefix + "tech", "DDR3-1333-CL10");  // see cpp file for other techs
    const char* addrMapping = config.get<const char*>(prefix + "addrMapping", "rank:col:bank");  // address splitter interleaves channels; row always on top

    // If set, writes are deferred and bursted out to reduce WTR overheads
    bool deferWrites = config.get<bool>(prefix + "deferWrites", true);
    bool closedPage = config.get<bool>(prefix + "closedPage", true);

    // Max row hits before we stop prioritizing further row hits to this bank.
    // Balances throughput and fairness; 0 -> FCFS / high (e.g., -1) -> pure FR-FCFS
    uint32_t maxRowHits = config.get<uint32_t>(prefix + "maxRowHits", 4);

    // Request queues
    uint32_t queueDepth = config.get<uint32_t>(prefix + "queueDepth", 16);
    uint32_t controllerLatency = config.get<uint32_t>(prefix + "controllerLatency", 10);  // in system cycles

    auto mem = new DDRMemory(zinfo->lineSize, pageSize, ranksPerChannel, banksPerRank, frequency, tech,
            addrMapping, controllerLatency, queueDepth, maxRowHits, deferWrites, closedPage, domain, name);
    return mem;
}

MemObject* BuildMemoryController(Config& config, uint32_t lineSize, uint32_t frequency, uint32_t domain, g_string& name) {
    //Type
    string type = config.get<const char*>("sys.mem.type", "Simple");

    //Latency
    uint32_t latency = (type == "DDR")? -1 : config.get<uint32_t>("sys.mem.latency", 100);

    MemObject* mem = nullptr;
    if (type == "Simple") {
        mem = new SimpleMemory(latency, name);
    } else if (type == "MD1") {
        // The following params are for MD1 only
        // NOTE: Frequency (in MHz) -- note this is a sys parameter (not sys.mem). There is an implicit assumption of having
        // a single CCT across the system, and we are dealing with latencies in *core* clock cycles

        // Peak bandwidth (in MB/s)
        uint32_t bandwidth = config.get<uint32_t>("sys.mem.bandwidth", 6400);

        mem = new MD1Memory(lineSize, frequency, bandwidth, latency, name);
    } else if (type == "WeaveMD1") {
        uint32_t bandwidth = config.get<uint32_t>("sys.mem.bandwidth", 6400);
        uint32_t boundLatency = config.get<uint32_t>("sys.mem.boundLatency", latency);
        mem = new WeaveMD1Memory(lineSize, frequency, bandwidth, latency, boundLatency, domain, name);
    } else if (type == "WeaveSimple") {
        uint32_t boundLatency = config.get<uint32_t>("sys.mem.boundLatency", 100);
        mem = new WeaveSimpleMemory(latency, boundLatency, domain, name);
    } else if (type == "DDR") {
        mem = BuildDDRMemory(config, lineSize, frequency, domain, name, "sys.mem.");
    } else if (type == "DRAMSim") {
        uint64_t cpuFreqHz = 1000000 * frequency;
        uint32_t capacity = config.get<uint32_t>("sys.mem.capacityMB", 16384);
        string dramTechIni = config.get<const char*>("sys.mem.techIni");
        string dramSystemIni = config.get<const char*>("sys.mem.systemIni");
        string outputDir = config.get<const char*>("sys.mem.outputDir");
        string traceName = config.get<const char*>("sys.mem.traceName");
        mem = new DRAMSimMemory(dramTechIni, dramSystemIni, outputDir, traceName, capacity, cpuFreqHz, latency, domain, name);
    } else if (type == "Detailed") {
        // FIXME(dsm): Don't use a separate config file... see DDRMemory
        g_string mcfg = config.get<const char*>("sys.mem.paramFile", "");
        mem = new MemControllerBase(mcfg, lineSize, frequency, domain, name);
    } else {
        panic("Invalid memory controller type %s", type.c_str());
    }
    return mem;
}

typedef vector<vector<BaseCache*>> CacheGroup;

CacheGroup* BuildCacheGroup(Config& config, const string& name, bool isTerminal) {
    CacheGroup* cgp = new CacheGroup;
    CacheGroup& cg = *cgp;

    string prefix = "sys.caches." + name + ".";

    uint32_t size = config.get<uint32_t>(prefix + "size", 64*1024);
    uint32_t banks = config.get<uint32_t>(prefix + "banks", 1);
    uint32_t caches = config.get<uint32_t>(prefix + "caches", 1);

    assert(((banks > 0) && (size  > 0)));

    uint32_t bankSize = size/banks;

    string prefetch_type = config.get<const char*>(prefix + "prefetcherType", "");
    if (!prefetch_type.empty()) { //build a prefetcher group
        if (prefetch_type.compare("Stride") == 0 ||
            prefetch_type.compare("GHB") == 0) {
            uint32_t prefetchers = config.get<uint32_t>(prefix + "prefetchers", 1);
            uint32_t streams = config.get<uint32_t>(prefix + "streams", 16);
            uint32_t log_distance = config.get<uint32_t>(prefix + "log_distance", 6);
            uint32_t degree = config.get<uint32_t>(prefix + "degree", 2);
            if(streams <= 0) {
                panic("Prefetcher requires at least one stream");
            }
            if(log_distance < 4 || log_distance > 31) {
                panic("Prefetch log distance must be in between 4 and 31");
            }
            cg.resize(prefetchers);
            for (vector<BaseCache*>& bg : cg) bg.resize(1);
            for (uint32_t i = 0; i < prefetchers; i++) {
                stringstream ss;
                ss << name << "-" << i;
                g_string pfName(ss.str().c_str());
                if (prefetch_type.compare("GHB") == 0) {
                    uint32_t ghb_size;
                    ghb_size = config.get<uint32_t>(prefix + "ghb_size", 1024);
                    uint32_t index_size =
                        config.get<uint32_t>(prefix + "index_size", 128);
                    g_string target_cache =
                        config.get<const char*>(prefix + "target", "");
                    bool delta_correlat =
                        config.get<bool>(prefix + "delta_correlation", false);
                    bool monitor_GETS =
                        config.get<bool>(prefix + "monitor_GETS", true);
                    bool monitor_GETX =
                        config.get<bool>(prefix + "monitor_GETX", true);
                    cg[i][0] = new GhbPrefetcher(pfName, ghb_size,
                                                 index_size, log_distance,
                                                 degree, target_cache,
                                                 delta_correlat, monitor_GETS,
                                                 monitor_GETX);
                }
                else {
                    if(!(degree > 0 && degree < 5)) {
                        panic("Prefetch degree must be in between 1 and 4");
                    }
                    cg[i][0] = new StreamPrefetcher(pfName, streams,
                                                    log_distance, degree);
                }
            }
            return cgp;
        } else if (prefetch_type.compare("Table") == 0) {
            g_string table_file = config.get<const char*>(prefix + "table", "");
            double conf_thresh = config.get<double>(prefix + "confidence", 0.0);
            double prob_thresh = config.get<double>(prefix + "probability", 0.0);
            bool monitor_loads = config.get<bool>(prefix + "monitorLoads", true);  //really 'GETS'
            bool monitor_stores = config.get<bool>(prefix + "monitorStores", true);  //really 'GETX'
            uint32_t degree = config.get<uint32_t>(prefix + "degree", 1);
            bool stride_detection = config.get<bool>(prefix + "strideDetection", false);
            g_string target_cache = config.get<const char*>(prefix + "target", "");
            uint32_t prefetchers = config.get<uint32_t>(prefix + "prefetchers", 1);
            if (table_file.empty()) {
                panic("Table prefetcher requires a table input file");
            }
            if (conf_thresh < 0.0 || conf_thresh > 1.0) {
                panic("Confidence threshold must be between 0 and 1");
            }
            if (prob_thresh < 0.0 || prob_thresh > 1.0) {
                panic("Probability threshold must be between 0 and 1");
            }
            if (target_cache.empty()) {
                panic("Unspecified target cache for prefetcher '%s'", name.c_str());
            }
            cg.resize(prefetchers);
            for (uint32_t i = 0; i < prefetchers; i++) {
                stringstream ss;
                ss << name << '-' << i;
                g_string full_name(ss.str().c_str());
                cg[i].emplace_back(new TablePrefetcher(full_name, target_cache, monitor_loads, monitor_stores,
                                                       table_file, conf_thresh, prob_thresh, degree, stride_detection));
            }
            return cgp;
        } else if (prefetch_type.compare("Nextline") == 0) {
            bool monitor_loads = config.get<bool>(prefix + "monitorLoads", true);  //really 'GETS'
            bool monitor_stores = config.get<bool>(prefix + "monitorStores", true);  //really 'GETX'
            uint32_t degree = config.get<uint32_t>(prefix + "degree", 0);
            g_string target_cache = config.get<const char*>(prefix + "target", "");
            uint32_t prefetchers = config.get<uint32_t>(prefix + "prefetchers", 1);
            if (target_cache.empty()) {
                panic("Unspecified target cache for prefetcher '%s'", name.c_str());
            }
            cg.resize(prefetchers);
            for (uint32_t i = 0; i < prefetchers; i++) {
                stringstream ss;
                ss << name << '-' << i;
                g_string full_name(ss.str().c_str());
                cg[i].emplace_back(new NextLinePrefetcher(full_name,
                                                          target_cache,
                                                          monitor_loads,
                                                          monitor_stores,
                                                          degree));
            }
            return cgp;
        } else {
            panic("Unknown prefetcher type '%s'", prefetch_type.c_str());
        }
    }
#ifdef ZSIM_USE_YT
    //Trace classes which are probes instead of block-holding caches
    string tracer_type = config.get<const char*>(prefix + "tracerType", "");
    if (!tracer_type.empty()) {
        assert(prefetch_type.empty());
        if(tracer_type.compare("YT") == 0) {
            string traceFile = config.get<const char*>(prefix + "traceFile","");
            bool demand = config.get<bool>(prefix + "demand", true);
            bool speculative = config.get<bool>(prefix + "speculative", true);
            cg.resize(1);
            cg[0].emplace_back(new YTTracer(name, traceFile, demand, speculative));
            return cgp;
        } else {
            panic("Unknown tracer type '%s'", tracer_type.c_str());
        }
    }
#endif  // ZSIM_USE_YT

    if (size % banks != 0) {
        panic("%s: banks (%d) does not divide the size (%d bytes)", name.c_str(), banks, size);
    }

    cg.resize(caches);
    for (vector<BaseCache*>& bg : cg) bg.resize(banks);

    for (uint32_t i = 0; i < caches; i++) {
        for (uint32_t j = 0; j < banks; j++) {
            stringstream ss;
            ss << name << "-" << i;
            if (banks > 1) {
                ss << "b" << j;
            }
            g_string bankName(ss.str().c_str());
            uint32_t domain = (i*banks + j)*zinfo->numDomains/(caches*banks); //(banks > 1)? nextDomain() : (i*banks + j)*zinfo->numDomains/(caches*banks);
            cg[i][j] = BuildCacheBank(config, prefix, bankName, bankSize, isTerminal, domain);
        }
    }

    return cgp;
}

static void InitSystem(Config& config) {
    unordered_map<string, string> parentMap; //child -> parent
    unordered_map<string, vector<vector<string>>> childMap; //parent -> children (a parent may have multiple children)

    auto parseChildren = [](string children) {
        // 1st dim: concatenated caches; 2nd dim: interleaved caches
        // Example: "l2-beefy l1i-wimpy|l1d-wimpy" produces [["l2-beefy"], ["l1i-wimpy", "l1d-wimpy"]]
        // If there are 2 of each cache, the final vector will be l2-beefy-0 l2-beefy-1 l1i-wimpy-0 l1d-wimpy-0 l1i-wimpy-1 l1d-wimpy-1
        vector<string> concatGroups = ParseList<string>(children);
        vector<vector<string>> cVec;
        for (string cg : concatGroups) cVec.push_back(ParseList<string>(cg, "|"));
        return cVec;
    };

    // If a network file is specified, build a Network
    string networkFile = config.get<const char*>("sys.networkFile", "");
    Network* network = (networkFile != "")? new Network(networkFile.c_str()) : nullptr;

    // Build the caches
    vector<const char*> cacheGroupNames;
    config.subgroups("sys.caches", cacheGroupNames);
    string prefix = "sys.caches.";

    for (const char* grp : cacheGroupNames) {
        string group(grp);
        if (group == "mem") panic("'mem' is an invalid cache group name");
        if (childMap.count(group)) panic("Duplicate cache group %s", (prefix + group).c_str());

        string children = config.get<const char*>(prefix + group + ".children", "");
        childMap[group] = parseChildren(children);
        for (auto v : childMap[group]) for (auto child : v) {
            if (parentMap.count(child)) {
                panic("Cache group %s can have only one parent (%s and %s found)", child.c_str(), parentMap[child].c_str(), grp);
            }
            parentMap[child] = group;
        }
    }

    // Check that children are valid (another cache)
    for (auto& it : parentMap) {
        bool found = false;
        for (auto& grp : cacheGroupNames) found |= it.first == grp;
        if (!found) panic("%s has invalid child %s", it.second.c_str(), it.first.c_str());
    }

    // Get the (single) LLC
    vector<string> parentlessCacheGroups;
    for (auto& it : childMap) if (!parentMap.count(it.first)) parentlessCacheGroups.push_back(it.first);
    if (parentlessCacheGroups.size() != 1) panic("Only one last-level cache allowed, found: %s", Str(parentlessCacheGroups).c_str());
    string llc = parentlessCacheGroups[0];

    auto isTerminal = [&](string group) -> bool {
        return childMap[group].size() == 0;
    };

    // Build each of the groups, starting with the LLC
    unordered_map<string, CacheGroup*> cMap;
    list<string> fringe;  // FIFO
    fringe.push_back(llc);
    while (!fringe.empty()) {
        string group = fringe.front();
        fringe.pop_front();
        if (cMap.count(group)) panic("The cache 'tree' has a loop at %s", group.c_str());
        cMap[group] = BuildCacheGroup(config, group, isTerminal(group));
        for (auto& childVec : childMap[group]) fringe.insert(fringe.end(), childVec.begin(), childVec.end());
    }

    //Check single LLC
    if (cMap[llc]->size() != 1) panic("Last-level cache %s must have caches = 1, but %ld were specified", llc.c_str(), cMap[llc]->size());

    /* Since we have checked for no loops, parent is mandatory, and all parents are checked valid,
     * it follows that we have a fully connected tree finishing at the LLC.
     */

    //Build the memory controllers
    uint32_t memControllers = config.get<uint32_t>("sys.mem.controllers", 1);
    assert(memControllers > 0);

    g_vector<MemObject*> mems;
    mems.resize(memControllers);

    for (uint32_t i = 0; i < memControllers; i++) {
        stringstream ss;
        ss << "mem-" << i;
        g_string name(ss.str().c_str());
        //uint32_t domain = nextDomain(); //i*zinfo->numDomains/memControllers;
        uint32_t domain = i*zinfo->numDomains/memControllers;
        mems[i] = BuildMemoryController(config, zinfo->lineSize, zinfo->freqMHz, domain, name);
    }

    if (memControllers > 1) {
        bool splitAddrs = config.get<bool>("sys.mem.splitAddrs", true);
        if (splitAddrs) {
            MemObject* splitter = new SplitAddrMemory(mems, "mem-splitter");
            mems.resize(1);
            mems[0] = splitter;
        }
    }

    //Connect everything
    bool printHierarchy = config.get<bool>("sim.printHierarchy", false);

    // mem to llc is a bit special, only one llc
    uint32_t childId = 0;
    for (BaseCache* llcBank : (*cMap[llc])[0]) {
        llcBank->setParents(childId++, mems, network);
    }

    // Rest of caches
    for (const char* grp : cacheGroupNames) {
        if (isTerminal(grp)) continue; //skip terminal caches

        CacheGroup& parentCaches = *cMap[grp];
        uint32_t parents = parentCaches.size();
        assert(parents);

        // Linearize concatenated / interleaved caches from childMap cacheGroups
        CacheGroup childCaches;

        for (auto childVec : childMap[grp]) {
            if (!childVec.size()) continue;
            size_t vecSize = cMap[childVec[0]]->size();
            for (string child : childVec) {
                if (cMap[child]->size() != vecSize) {
                    panic("In interleaved group %s, %s has a different number of caches", Str(childVec).c_str(), child.c_str());
                }
            }

            CacheGroup interleavedGroup;
            for (uint32_t i = 0; i < vecSize; i++) {
                for (uint32_t j = 0; j < childVec.size(); j++) {
                    interleavedGroup.push_back(cMap[childVec[j]]->at(i));
                }
            }

            childCaches.insert(childCaches.end(), interleavedGroup.begin(), interleavedGroup.end());
        }

        uint32_t children = childCaches.size();
        assert(children);

        uint32_t childrenPerParent = children/parents;
        if (children % parents != 0) {
            panic("%s has %d caches and %d children, they are non-divisible. "
                  "Use multiple groups for non-homogeneous children per parent!", grp, parents, children);
        }

        for (uint32_t p = 0; p < parents; p++) {
            g_vector<MemObject*> parentsVec;
            parentsVec.insert(parentsVec.end(), parentCaches[p].begin(), parentCaches[p].end()); //BaseCache* to MemObject* is a safe cast

            uint32_t childId = 0;
            g_vector<BaseCache*> childrenVec;
            for (uint32_t c = p*childrenPerParent; c < (p+1)*childrenPerParent; c++) {
                for (BaseCache* bank : childCaches[c]) {
                    bank->setParents(childId++, parentsVec, network);
                    childrenVec.push_back(bank);
                }
            }

            if (printHierarchy) {
                vector<string> cacheNames;
                std::transform(childrenVec.begin(), childrenVec.end(), std::back_inserter(cacheNames),
                        [](BaseCache* c) -> string { string s = c->getName(); return s; });

                string parentName = parentCaches[p][0]->getName();
                if (parentCaches[p].size() > 1) {
                    parentName += "..";
                    parentName += parentCaches[p][parentCaches[p].size()-1]->getName();
                }
                info("Hierarchy: %s -> %s", Str(cacheNames).c_str(), parentName.c_str());
            }

            for (BaseCache* bank : parentCaches[p]) {
                bank->setChildren(childrenVec, network);
            }
        }
    }

    //Check that all the terminal caches have a single bank
    for (const char* grp : cacheGroupNames) {
        if (isTerminal(grp)) {
            uint32_t banks = (*cMap[grp])[0].size();
            if (banks != 1) panic("Terminal cache group %s needs to have a single bank, has %d", grp, banks);
        }
    }

    //Tracks how many terminal caches have been allocated to cores
    unordered_map<string, uint32_t> assignedCaches;
    for (const char* grp : cacheGroupNames) if (isTerminal(grp)) assignedCaches[grp] = 0;

    if (!zinfo->traceDriven) {
        //Instantiate the cores
        vector<const char*> coreGroupNames;
        unordered_map <string, vector<Core*>> coreMap;
        config.subgroups("sys.cores", coreGroupNames);

        uint32_t coreIdx = 0;
        for (const char* group : coreGroupNames) {
            if (parentMap.count(group)) panic("Core group name %s is invalid, a cache group already has that name", group);

            coreMap[group] = vector<Core*>();

            string prefix = string("sys.cores.") + group + ".";
            uint32_t cores = config.get<uint32_t>(prefix + "cores", 1);
            string type = config.get<const char*>(prefix + "type", "Simple");
            string prefixc = prefix + "properties.";
            uint32_t bp_nb = config.get<uint32_t>(prefixc + "bp_nb", 11);
            uint32_t bp_hb = config.get<uint32_t>(prefixc + "bp_hb", 18);
            uint32_t bp_lb = config.get<uint32_t>(prefixc + "bp_lb", 14);

            CoreProperties *core_properties = new CoreProperties(bp_nb, bp_hb, bp_lb);


            //Build the core group
            union {
                SimpleCore* simpleCores;
                TimingCore* timingCores;
                OOOCore* oooCores;
                NullCore* nullCores;
            };
            if (type == "Simple") {
                simpleCores = gm_memalign<SimpleCore>(CACHE_LINE_BYTES, cores);
            } else if (type == "Timing") {
                timingCores = gm_memalign<TimingCore>(CACHE_LINE_BYTES, cores);
            } else if (type == "OOO") {
                oooCores = gm_memalign<OOOCore>(CACHE_LINE_BYTES, cores);
                zinfo->oooDecode = true; //enable uop decoding, this is false by default, must be true if even one OOO cpu is in the system
            } else if (type == "Null") {
                nullCores = gm_memalign<NullCore>(CACHE_LINE_BYTES, cores);
            } else {
                panic("%s: Invalid core type %s", group, type.c_str());
            }

            if (type != "Null") {
                string icache = config.get<const char*>(prefix + "icache");
                string dcache = config.get<const char*>(prefix + "dcache");

                if (!assignedCaches.count(icache)) panic("%s: Invalid icache parameter %s", group, icache.c_str());
                if (!assignedCaches.count(dcache)) panic("%s: Invalid dcache parameter %s", group, dcache.c_str());

                for (uint32_t j = 0; j < cores; j++) {
                    stringstream ss;
                    ss << group << "-" << j;
                    g_string name(ss.str().c_str());
                    Core* core;

                    //Get the caches
                    CacheGroup& igroup = *cMap[icache];
                    CacheGroup& dgroup = *cMap[dcache];

                    if (assignedCaches[icache] >= igroup.size()) {
                        panic("%s: icache group %s (%ld caches) is fully used, can't connect more cores to it", name.c_str(), icache.c_str(), igroup.size());
                    }
                    FilterCache* ic = dynamic_cast<FilterCache*>(igroup[assignedCaches[icache]][0]);
                    assert(ic);
                    ic->setSourceId(coreIdx);
                    ic->setFlags(MemReq::IFETCH | MemReq::NOEXCL);
                    ic->setType(FilterCache::Type::I);
                    assignedCaches[icache]++;

                    if (assignedCaches[dcache] >= dgroup.size()) {
                        panic("%s: dcache group %s (%ld caches) is fully used, can't connect more cores to it", name.c_str(), dcache.c_str(), dgroup.size());
                    }
                    FilterCache* dc = dynamic_cast<FilterCache*>(dgroup[assignedCaches[dcache]][0]);
                    assert(dc);
                    dc->setSourceId(coreIdx);
                    dc->setType(FilterCache::Type::D);
                    assignedCaches[dcache]++;

                    //Build the core
                    if (type == "Simple") {
                        core = new (&simpleCores[j]) SimpleCore(ic, dc, name);
                    } else if (type == "Timing") {
                        uint32_t domain = j*zinfo->numDomains/cores;
                        TimingCore* tcore = new (&timingCores[j]) TimingCore(ic, dc, domain, name);
                        zinfo->eventRecorders[coreIdx] = tcore->getEventRecorder();
                        zinfo->eventRecorders[coreIdx]->setSourceId(coreIdx);
                        core = tcore;
                    } else {
                        assert(type == "OOO");
                        OOOCore* ocore = new (&oooCores[j]) OOOCore(ic, dc, name, core_properties);
                        zinfo->eventRecorders[coreIdx] = ocore->getEventRecorder();
                        zinfo->eventRecorders[coreIdx]->setSourceId(coreIdx);
                        core = ocore;
                    }
                    coreMap[group].push_back(core);
                    coreIdx++;
                }
            } else {
                assert(type == "Null");
                for (uint32_t j = 0; j < cores; j++) {
                    stringstream ss;
                    ss << group << "-" << j;
                    g_string name(ss.str().c_str());
                    Core* core = new (&nullCores[j]) NullCore(name);
                    coreMap[group].push_back(core);
                    coreIdx++;
                }
            }
        }

        //Check that all the terminal caches are fully connected
        for (const char* grp : cacheGroupNames) {
            if (isTerminal(grp) && assignedCaches[grp] != cMap[grp]->size()) {
                panic("%s: Terminal cache group not fully connected, %ld caches, %d assigned", grp, cMap[grp]->size(), assignedCaches[grp]);
            }
        }

        //Populate global core info
        assert(zinfo->numCores == coreIdx);
        zinfo->cores = gm_memalign<Core*>(CACHE_LINE_BYTES, zinfo->numCores);
        zinfo->readers = gm_memalign<TraceReader*>(CACHE_LINE_BYTES, zinfo->numCores);
        coreIdx = 0;
        for (const char* group : coreGroupNames) for (Core* core : coreMap[group]) zinfo->cores[coreIdx++] = core;

        //Init stats: cores
        for (const char* group : coreGroupNames) {
            AggregateStat* groupStat = new AggregateStat(true);
            groupStat->init(gm_strdup(group), "Core stats");
            for (Core* core : coreMap[group]) core->initStats(groupStat);
            zinfo->rootStat->append(groupStat);
        }
    } else {  // trace-driven: create trace driver and proxy caches
        vector<TraceDriverProxyCache*> proxies;
        for (const char* grp : cacheGroupNames) {
            if (isTerminal(grp)) {
                for (vector<BaseCache*> cv : *cMap[grp]) {
                    assert(cv.size() == 1);
                    TraceDriverProxyCache* proxy = dynamic_cast<TraceDriverProxyCache*>(cv[0]);
                    assert(proxy);
                    proxies.push_back(proxy);
                }
            }
        }

        //FIXME: For now, we assume we are driving a single-bank LLC
        string traceFile = config.get<const char*>("sim.traceFile");
        string retraceFile = config.get<const char*>("sim.retraceFile", ""); //leave empty to not retrace
        zinfo->traceDriver = new TraceDriver(traceFile, retraceFile, proxies,
                config.get<bool>("sim.useSkews", true), // incorporate skews in to playback and simulator results, not only the output trace
                config.get<bool>("sim.playPuts", true),
                config.get<bool>("sim.playAllGets", true));
        zinfo->traceDriver->initStats(zinfo->rootStat);
    }

    //Call postInit() on every cache. For most caches this will do nothing, but some may wish to take some action that
    //is only available at this point, such as traversing the memory hierarchy.
    for (auto& group_pair : cMap) {
        for (auto& bank_vector : *(group_pair.second)) {
            for (BaseCache* cache : bank_vector) {
                cache->postInit();
            }
        }
    }

    //Init stats: caches, mem
    for (const char* group : cacheGroupNames) {
        AggregateStat* groupStat = new AggregateStat(true);
        groupStat->init(gm_strdup(group), "Cache stats");
        for (vector<BaseCache*>& banks : *cMap[group]) for (BaseCache* bank : banks) bank->initStats(groupStat);
        zinfo->rootStat->append(groupStat);
    }

    //Initialize event recorders
    //for (uint32_t i = 0; i < zinfo->numCores; i++) eventRecorders[i] = new EventRecorder();

    AggregateStat* memStat = new AggregateStat(true);
    memStat->init("mem", "Memory controller stats");
    for (auto mem : mems) mem->initStats(memStat);
    zinfo->rootStat->append(memStat);

    //Odds and ends: BuildCacheGroup new'd the cache groups, we need to delete them
    for (pair<string, CacheGroup*> kv : cMap) delete kv.second;
    cMap.clear();

    info("Initialized system");
}

static void PreInitStats() {
    zinfo->rootStat = new AggregateStat();
    zinfo->rootStat->init("root", "Stats");
}

static void PostInitStats(bool perProcessDir, Config& config) {
    zinfo->rootStat->makeImmutable();
    zinfo->trigger = 15000;

    string pathStr = zinfo->outputDir;
    pathStr += "/";

    // Absolute paths for stats files. Note these must be in the global heap.
    const char* pStatsFile = gm_strdup((pathStr + "zsim.h5").c_str());
    const char* evStatsFile = gm_strdup((pathStr + "zsim-ev.h5").c_str());
    const char* cmpStatsFile = gm_strdup((pathStr + "zsim-cmp.h5").c_str());
    const char* statsFile = gm_strdup((pathStr + "zsim.out").c_str());

    if (zinfo->statsPhaseInterval) {
        const char* periodicStatsFilter = config.get<const char*>("sim.periodicStatsFilter", "");
        AggregateStat* prStat = (!strlen(periodicStatsFilter))? zinfo->rootStat : FilterStats(zinfo->rootStat, periodicStatsFilter);
        if (!prStat) panic("No stats match sim.periodicStatsFilter regex (%s)! Set interval to 0 to avoid periodic stats", periodicStatsFilter);
        zinfo->periodicStatsBackend = new HDF5Backend(pStatsFile, prStat, (1 << 20) /* 1MB chunks */, zinfo->skipStatsVectors, zinfo->compactPeriodicStats);
        zinfo->periodicStatsBackend->dump(true); //must have a first sample

        class PeriodicStatsDumpEvent : public Event {
            public:
                explicit PeriodicStatsDumpEvent(uint32_t period) : Event(period) {}
                void callback() {
                    zinfo->trigger = 10000;
                    zinfo->periodicStatsBackend->dump(true /*buffered*/);
                }
        };

        zinfo->eventQueue->insert(new PeriodicStatsDumpEvent(zinfo->statsPhaseInterval));
        zinfo->statsBackends->push_back(zinfo->periodicStatsBackend);
    } else {
        zinfo->periodicStatsBackend = nullptr;
    }

    zinfo->eventualStatsBackend = new HDF5Backend(evStatsFile, zinfo->rootStat, (1 << 17) /* 128KB chunks */, zinfo->skipStatsVectors, false /* don't sum regular aggregates*/);
    zinfo->eventualStatsBackend->dump(true); //must have a first sample
    zinfo->statsBackends->push_back(zinfo->eventualStatsBackend);

    if (zinfo->maxMinInstrs) {
        warn("maxMinInstrs IS DEPRECATED");
        for (uint32_t i = 0; i < zinfo->numCores; i++) {
            auto getInstrs = [i]() { return zinfo->cores[i]->getInstrs(); };
            auto dumpStats = [i]() {
                info("Dumping eventual stats for core %d", i);
                zinfo->trigger = i;
                zinfo->eventualStatsBackend->dump(true /*buffered*/);
            };
            zinfo->eventQueue->insert(makeAdaptiveEvent(getInstrs, dumpStats, 0, zinfo->maxMinInstrs, MAX_IPC*zinfo->phaseLength));
        }
    }

    // Convenience stats
    StatsBackend* compactStats = new HDF5Backend(cmpStatsFile, zinfo->rootStat, 0 /* no aggregation, this is just 1 record */, zinfo->skipStatsVectors, true); //don't dump a first sample.
    StatsBackend* textStats = new TextBackend(statsFile, zinfo->rootStat);
    zinfo->statsBackends->push_back(compactStats);
    zinfo->statsBackends->push_back(textStats);
}

static void InitGlobalStats() {
    zinfo->profSimTime = new TimeBreakdownStat();
    const char* stateNames[] = {"init", "bound", "weave", "ff"};
    zinfo->profSimTime->init("time", "Simulator time breakdown", 4, stateNames);
    zinfo->rootStat->append(zinfo->profSimTime);

    ProxyStat* triggerStat = new ProxyStat();
    triggerStat->init("trigger", "Reason for this stats dump", &zinfo->trigger);
    zinfo->rootStat->append(triggerStat);

    ProxyStat* phaseStat = new ProxyStat();
    phaseStat->init("phase", "Simulated phases", &zinfo->numPhases);
    zinfo->rootStat->append(phaseStat);
}


void SimInit(const char* configFile, const char* outputDir, uint32_t shmid) {
    zinfo = gm_calloc<GlobSimInfo>();
    zinfo->outputDir = gm_strdup(outputDir);
    zinfo->statsBackends = new g_vector<StatsBackend*>();

    Config config(configFile);

    PreInitStats();

    zinfo->traceDriven = config.get<bool>("sim.traceDriven", false);

    if (zinfo->traceDriven) {
        zinfo->numCores = 0;
    } else {
        // Get the number of cores
        // TODO: There is some duplication with the core creation code. This should be fixed eventually.
        uint32_t numCores = 0;
        vector<const char*> groups;
        config.subgroups("sys.cores", groups);
        for (const char* group : groups) {
            uint32_t cores = config.get<uint32_t>(string("sys.cores.") + group + ".cores", 1);
            numCores += cores;
        }

        if (numCores == 0) panic("Config must define some core classes in sys.cores; sys.numCores is deprecated");
        zinfo->numCores = numCores;
        assert(numCores <= MAX_THREADS); //TODO: Is there any reason for this limit?
    }

    zinfo->numDomains = config.get<uint32_t>("sim.domains", 1);
    uint32_t numSimThreads = config.get<uint32_t>("sim.contentionThreads", MAX((uint32_t)1, zinfo->numDomains/2)); //gives a bit of parallelism, TODO tune
    zinfo->contentionSim = new ContentionSim(zinfo->numDomains, numSimThreads);
    zinfo->contentionSim->initStats(zinfo->rootStat);
    zinfo->eventRecorders = gm_calloc<EventRecorder*>(zinfo->numCores);

    zinfo->traceWriters = new g_vector<AccessTraceWriter*>();
#ifdef ZSIM_USE_YT
    zinfo->YTtraceWriters = new g_vector<YTTracer*>();
#endif  // ZSIM_USE_YT

    // Global simulation values
    zinfo->numPhases = 0;

    zinfo->phaseLength = config.get<uint32_t>("sim.phaseLength", 10000);
    zinfo->statsPhaseInterval = config.get<uint32_t>("sim.statsPhaseInterval", 100);
    zinfo->freqMHz = config.get<uint32_t>("sys.frequency", 2000);

    //Maxima/termination conditions
    zinfo->maxPhases = config.get<uint64_t>("sim.maxPhases", 0);
    zinfo->maxMinInstrs = config.get<uint64_t>("sim.maxMinInstrs", 0);
    zinfo->maxTotalInstrs = config.get<uint64_t>("sim.maxTotalInstrs", 0);

    uint64_t maxSimTime = config.get<uint32_t>("sim.maxSimTime", 0);
    zinfo->maxSimTimeNs = maxSimTime*1000L*1000L*1000L;

    zinfo->maxProcEventualDumps = config.get<uint32_t>("sim.maxProcEventualDumps", 0);
    zinfo->procEventualDumps = 0;

    zinfo->skipStatsVectors = config.get<bool>("sim.skipStatsVectors", false);
    zinfo->compactPeriodicStats = config.get<bool>("sim.compactPeriodicStats", false);

    //Fast-forwarding and magic ops
    zinfo->ignoreHooks = config.get<bool>("sim.ignoreHooks", false);
    zinfo->ffReinstrument = config.get<bool>("sim.ffReinstrument", false);
    if (zinfo->ffReinstrument) warn("sim.ffReinstrument = true, switching fast-forwarding on a multi-threaded process may be unstable");

    zinfo->registerThreads = config.get<bool>("sim.registerThreads", false);
    zinfo->globalPauseFlag = config.get<bool>("sim.startInGlobalPause", false);

    zinfo->eventQueue = new EventQueue(); //must be instantiated before the memory hierarchy

    InitGlobalStats();

    //Core stats (initialized here for cosmetic reasons, to be above cache stats)
    AggregateStat* allCoreStats = new AggregateStat(false);
    allCoreStats->init("core", "Core stats");
    zinfo->rootStat->append(allCoreStats);

    //Process tree needs this initialized, even though it is part of the memory hierarchy
    zinfo->lineSize = config.get<uint32_t>("sys.lineSize", 64);
    assert(zinfo->lineSize > 0);

    //Caches, cores, memory controllers
    InitSystem(config);

    //It's a global stat, but I want it to be last...
    zinfo->profHeartbeats = new VectorCounter();
    zinfo->profHeartbeats->init("heartbeats", "Per-process heartbeats", zinfo->lineSize);
    zinfo->rootStat->append(zinfo->profHeartbeats);

    bool perProcessDir = config.get<bool>("sim.perProcessDir", false);
    PostInitStats(perProcessDir, config);

    zinfo->perProcessCpuEnum = config.get<bool>("sim.perProcessCpuEnum", false);

    //HACK: Read all variables that are read in the harness but not in init
    //This avoids warnings on those elements
    config.get<uint32_t>("sim.gmMBytes", (1 << 10));
    if (!zinfo->attachDebugger) config.get<bool>("sim.deadlockDetection", true);
    config.get<bool>("sim.aslr", false);
    config.get<const char*>("sim.outputDir", nullptr);
    config.get<const char*>("trace_binaries", nullptr);
    config.get<const char*>("trace_type", nullptr);
    for (uint32_t i = 0; i < zinfo->numCores; i++) {
        std::stringstream p_ss;
        p_ss << "trace" << i;
        config.get<const char*>(p_ss.str(), nullptr);
    }

    //Write config out
    bool strictConfig = config.get<bool>("sim.strictConfig", true); //if true, panic on unused variables
    config.writeAndClose((string(zinfo->outputDir) + "/out.cfg").c_str(), strictConfig);

    zinfo->contentionSim->postInit();

    info("Initialization complete");

}
