#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/energy-module.h"
#include "ns3/netanim-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

using namespace ns3;
using namespace ns3::energy;

NS_LOG_COMPONENT_DEFINE("LeachPerformance");

static bool g_forceCH = false;
static std::vector<uint32_t> g_forcedCHIds;
static std::string g_protocol = "LEACH";
static NodeContainer* g_sensorNodesForElection = nullptr;
static EnergySourceContainer* g_energyForElection = nullptr;


static std::vector<uint32_t>
SelectClusterHeads(uint32_t round,
                   uint32_t nSensors,
                   double probability,
                   std::vector<bool>& eligible,
                   Ptr<UniformRandomVariable> randomVariable)
{
    std::vector<uint32_t> clusterHeads;

    // Direct transmission has no cluster heads.
    if (g_protocol == "Direct")
    {
        return clusterHeads;
    }

    // Centralized C-LEACH: BS ranks alive sensors using residual energy and
    // distance to BS, then selects approximately P*N cluster heads.
    if (g_protocol == "C-LEACH")
    {
        std::vector<std::pair<double, uint32_t>> ranked;
        const uint32_t target = std::max<uint32_t>(1,
            static_cast<uint32_t>(std::round(probability * nSensors)));
        for (uint32_t i = 0; i < nSensors; ++i)
        {
            Ptr<BasicEnergySource> src = DynamicCast<BasicEnergySource>(g_energyForElection->Get(i));
            const double energy = src ? src->GetRemainingEnergy() : 0.0;
            if (energy <= 0.0) continue;
            const Vector pos = g_sensorNodesForElection->Get(i)->GetObject<MobilityModel>()->GetPosition();
            const double dx = pos.x - 50.0;
            const double dy = pos.y - 120.0;
            const double distance = std::sqrt(dx*dx + dy*dy);
            const double score = 0.7 * (energy / 100.0) + 0.3 / (1.0 + distance);
            ranked.emplace_back(score, i);
        }
        std::sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) {
            return a.first > b.first;
        });
        for (uint32_t k = 0; k < std::min<uint32_t>(target, ranked.size()); ++k)
            clusterHeads.push_back(ranked[k].second);
        return clusterHeads;
    }

    if (g_forceCH)
    {
        std::set<uint32_t> uniqueIds;
        for (uint32_t id : g_forcedCHIds)
        {
            if (id >= nSensors)
                NS_FATAL_ERROR("Forced CH ID out of range: " << id);
            if (!uniqueIds.insert(id).second)
                NS_FATAL_ERROR("Duplicate forced CH ID: " << id);
            clusterHeads.push_back(id);
        }
        if (clusterHeads.empty())
            NS_FATAL_ERROR("Forced CH set cannot be empty");
        return clusterHeads;
    }

    const uint32_t epochLength =
        std::max<uint32_t>(1, static_cast<uint32_t>(1.0 / probability));

    if (round > 0 && round % epochLength == 0)
    {
        for (uint32_t i = 0; i < nSensors; ++i)
        {
            eligible[i] = true;
        }
        std::cout << "\n*** Starting new LEACH epoch ***\n";
    }

    const double denominator = 1.0 - probability * (round % epochLength);
    const double threshold = probability / denominator;

    for (uint32_t i = 0; i < nSensors; ++i)
    {
        if (!eligible[i])
        {
            continue;
        }
        const double randomNumber = randomVariable->GetValue(0.0, 1.0);
        if (randomNumber < threshold)
        {
            clusterHeads.push_back(i);
            eligible[i] = false;
        }
    }
    return clusterHeads;
}

static std::vector<uint32_t>
ParseForcedCHIds(const std::string& text)
{
    std::vector<uint32_t> ids;
    std::stringstream ss(text);
    std::string token;
    while (std::getline(ss, token, ','))
    {
        if (token.empty())
            NS_FATAL_ERROR("Empty token in forcedCHIds");
        size_t pos = 0;
        unsigned long value = 0;
        try { value = std::stoul(token, &pos); }
        catch (const std::exception&) { NS_FATAL_ERROR("Invalid forced CH ID: " << token); }
        if (pos != token.size() || value > std::numeric_limits<uint32_t>::max())
            NS_FATAL_ERROR("Invalid forced CH ID: " << token);
        ids.push_back(static_cast<uint32_t>(value));
    }
    std::set<uint32_t> uniqueIds(ids.begin(), ids.end());
    if (ids.empty() || uniqueIds.size() != ids.size())
        NS_FATAL_ERROR("Forced CH list is empty or contains duplicates");
    return ids;
}

static std::vector<int>
AssignClusters(NodeContainer& sensorNodes,
               const std::vector<uint32_t>& clusterHeads)
{
    std::vector<int> assignment(sensorNodes.GetN(), -1);
    if (clusterHeads.empty())
    {
        return assignment;
    }

    for (uint32_t i = 0; i < sensorNodes.GetN(); ++i)
    {
        if (std::find(clusterHeads.begin(), clusterHeads.end(), i) != clusterHeads.end())
        {
            assignment[i] = static_cast<int>(i);
            continue;
        }

        const Vector pos = sensorNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        double minDistance = std::numeric_limits<double>::max();
        int nearest = -1;

        for (uint32_t ch : clusterHeads)
        {
            const Vector chPos =
                sensorNodes.Get(ch)->GetObject<MobilityModel>()->GetPosition();
            const double dx = pos.x - chPos.x;
            const double dy = pos.y - chPos.y;
            const double distance = std::sqrt(dx * dx + dy * dy);
            if (distance < minDistance)
            {
                minDistance = distance;
                nearest = static_cast<int>(ch);
            }
        }
        assignment[i] = nearest;
    }
    return assignment;
}

static void
UpdateClusterHeadColors(AnimationInterface* anim,
                        NodeContainer sensorNodes,
                        const std::vector<uint32_t>& clusterHeads)
{
    for (uint32_t i = 0; i < sensorNodes.GetN(); ++i)
    {
        anim->UpdateNodeColor(sensorNodes.Get(i), 0, 0, 255);
        anim->UpdateNodeDescription(sensorNodes.Get(i), "Sensor-" + std::to_string(i));
    }
    for (uint32_t ch : clusterHeads)
    {
        anim->UpdateNodeColor(sensorNodes.Get(ch), 0, 255, 0);
        anim->UpdateNodeDescription(sensorNodes.Get(ch), "CH-" + std::to_string(ch));
    }
}

class LeachRoundController
{
  public:
    LeachRoundController(uint32_t nSensors,
                         uint32_t nRounds,
                         double roundDuration,
                         double chProbability,
                         uint16_t port,
                         NodeContainer sensorNodes,
                         Ipv4InterfaceContainer interfaces,
                         EnergySourceContainer energySources,
                         std::vector<bool> eligible,
                         Ptr<UniformRandomVariable> randomVariable,
                         AnimationInterface* anim,
                         FlowMonitorHelper* flowHelper,
                         Ptr<FlowMonitor> flowMonitor,
                         std::string scenarioId,
                         std::string stateCsv,
                         std::string protocol)
        : m_nSensors(nSensors),
          m_nRounds(nRounds),
          m_roundDuration(roundDuration),
          m_chProbability(chProbability),
          m_port(port),
          m_sensorNodes(sensorNodes),
          m_interfaces(interfaces),
          m_energySources(energySources),
          m_eligible(eligible),
          m_randomVariable(randomVariable),
          m_anim(anim),
          m_flowHelper(flowHelper),
          m_flowMonitor(flowMonitor),
          m_scenarioId(scenarioId),
          m_stateCsv(stateCsv),
          m_protocol(protocol),
          m_headerWritten(false),
          m_previousTxBytes(nSensors, 0),
          m_previousRxBytes(nSensors, 0),
          m_sensorPacketsScheduled(0),
          m_chPacketsScheduled(0)
    {
    }

    void Start()
    {
        // RunRound is an event, so state is sampled at the actual round boundary.
        Simulator::Schedule(Seconds(0.0), &LeachRoundController::RunRound, this, 0);
    }

  private:
    void WriteNodeState(uint32_t round)
    {
        std::vector<uint64_t> cumulativeTx(m_nSensors, 0);
        std::vector<uint64_t> cumulativeRx(m_nSensors, 0);

        // FlowMonitor values are cumulative. Attribute bytes to each sensor using
        // the flow source/destination IP addresses, then difference against the
        // previous round boundary to obtain per-round-window traffic.
        Ptr<Ipv4FlowClassifier> classifier =
            DynamicCast<Ipv4FlowClassifier>(m_flowHelper->GetClassifier());
        const auto stats = m_flowMonitor->GetFlowStats();

        for (const auto& item : stats)
        {
            const auto tuple = classifier->FindFlow(item.first);
            const auto& fs = item.second;

            for (uint32_t i = 0; i < m_nSensors; ++i)
            {
                const Ipv4Address ip = m_interfaces.GetAddress(i);
                if (tuple.sourceAddress == ip)
                {
                    cumulativeTx[i] += fs.txBytes;
                }
                if (tuple.destinationAddress == ip)
                {
                    cumulativeRx[i] += fs.rxBytes;
                }
            }
        }

        std::ofstream out(m_stateCsv, std::ios::app);
        if (!out.is_open())
        {
            NS_FATAL_ERROR("Cannot open node-state CSV: " << m_stateCsv);
        }
        if (!m_headerWritten)
        {
            // In append mode, avoid writing a second header to a nonempty file.
            std::ifstream check(m_stateCsv);
            const bool wasNonEmpty = check.peek() != std::ifstream::traits_type::eof();
            check.close();
            if (!wasNonEmpty)
            {
                out << "scenario_id,state_id,round,node_id,x_m,y_m,residual_energy_j,"
                       "distance_bs_m,traffic_tx_bytes_window,traffic_rx_bytes_window,eligible_ch\n";
            }
            m_headerWritten = true;
        }

        // Base station is the final interface in the interface container.
        const uint32_t bsInterfaceIndex = m_nSensors;
        const Ipv4Address bsIp = m_interfaces.GetAddress(bsInterfaceIndex);
        Vector bsPosition(50.0, 120.0, 0.0);

        // The fixed base station coordinates are set in main() to (50,120,0).
        (void)bsIp;

        for (uint32_t i = 0; i < m_nSensors; ++i)
        {
            const Vector p =
                m_sensorNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();

            Ptr<BasicEnergySource> source =
                DynamicCast<BasicEnergySource>(m_energySources.Get(i));
            const double remaining = source ? source->GetRemainingEnergy() : 0.0;

            const double dx = p.x - bsPosition.x;
            const double dy = p.y - bsPosition.y;
            const double distance = std::sqrt(dx * dx + dy * dy);

            const uint64_t txWindow =
                cumulativeTx[i] >= m_previousTxBytes[i]
                    ? cumulativeTx[i] - m_previousTxBytes[i]
                    : cumulativeTx[i];
            const uint64_t rxWindow =
                cumulativeRx[i] >= m_previousRxBytes[i]
                    ? cumulativeRx[i] - m_previousRxBytes[i]
                    : cumulativeRx[i];

            // This flag means alive (positive remaining energy), not classic
            // LEACH epoch eligibility. Document/adjust an energy threshold if used.
            const uint32_t aliveEligible = remaining > 0.0 ? 1 : 0;

            out << m_scenarioId << ","
                << m_scenarioId << "_r" << round << ","
                << round << ","
                << i << ","
                << std::setprecision(12) << p.x << ","
                << p.y << ","
                << remaining << ","
                << distance << ","
                << txWindow << ","
                << rxWindow << ","
                << aliveEligible << "\n";
        }

        m_previousTxBytes = cumulativeTx;
        m_previousRxBytes = cumulativeRx;
    }

    void RunRound(uint32_t round)
    {
        if (round >= m_nRounds)
        {
            return;
        }

        // Log the actual state at this round boundary, before selecting CHs.
        WriteNodeState(round);

        std::vector<uint32_t> clusterHeads =
            SelectClusterHeads(round, m_nSensors, m_chProbability,
                             m_eligible, m_randomVariable);

        const std::vector<int> assignment = AssignClusters(m_sensorNodes, clusterHeads);
        UpdateClusterHeadColors(m_anim, m_sensorNodes, clusterHeads);

        const Ipv4Address bsAddress = m_interfaces.GetAddress(m_nSensors);
        if (m_protocol == "Direct")
        {
            // Direct: every live sensor sends one packet directly to the BS.
            for (uint32_t i = 0; i < m_nSensors; ++i)
            {
                Ptr<BasicEnergySource> src = DynamicCast<BasicEnergySource>(m_energySources.Get(i));
                if (!src || src->GetRemainingEnergy() <= 0.0) continue;
                UdpClientHelper client(bsAddress, m_port);
                client.SetAttribute("MaxPackets", UintegerValue(1));
                client.SetAttribute("Interval", TimeValue(Seconds(1.0)));
                client.SetAttribute("PacketSize", UintegerValue(100));
                ApplicationContainer app = client.Install(m_sensorNodes.Get(i));
                app.Start(Simulator::Now() + Seconds(1.0));
                app.Stop(Simulator::Now() + Seconds(1.5));
                ++m_sensorPacketsScheduled;
            }
        }
        else
        {
            // LEACH and C-LEACH: members send to their nearest selected CH.
            for (uint32_t i = 0; i < m_nSensors; ++i)
            {
                if (assignment[i] < 0 || assignment[i] == static_cast<int>(i)) continue;
                UdpClientHelper client(m_interfaces.GetAddress(static_cast<uint32_t>(assignment[i])), m_port);
                client.SetAttribute("MaxPackets", UintegerValue(1));
                client.SetAttribute("Interval", TimeValue(Seconds(1.0)));
                client.SetAttribute("PacketSize", UintegerValue(100));
                ApplicationContainer app = client.Install(m_sensorNodes.Get(i));
                app.Start(Simulator::Now() + Seconds(1.0));
                app.Stop(Simulator::Now() + Seconds(1.5));
                ++m_sensorPacketsScheduled;
            }
            // CH -> BS: one packet per selected CH (baseline forwarding traffic).
            for (uint32_t ch : clusterHeads)
            {
                Ptr<BasicEnergySource> src = DynamicCast<BasicEnergySource>(m_energySources.Get(ch));
                if (!src || src->GetRemainingEnergy() <= 0.0) continue;
                UdpClientHelper client(bsAddress, m_port);
                client.SetAttribute("MaxPackets", UintegerValue(1));
                client.SetAttribute("Interval", TimeValue(Seconds(1.0)));
                client.SetAttribute("PacketSize", UintegerValue(100));
                ApplicationContainer app = client.Install(m_sensorNodes.Get(ch));
                app.Start(Simulator::Now() + Seconds(3.0));
                app.Stop(Simulator::Now() + Seconds(3.5));
                ++m_chPacketsScheduled;
            }
        }

        const uint32_t epochLength =
            std::max<uint32_t>(1, static_cast<uint32_t>(1.0 / m_chProbability));
        const double threshold = m_protocol == "LEACH"
            ? m_chProbability / (1.0 - m_chProbability * (round % epochLength)) : 0.0;

        std::cout << "\n====================================\n"
                  << m_protocol << " Round: " << round + 1 << "\n"
                  << "====================================\n"
                  << "Threshold: " << threshold << "\n"
                  << "Cluster Heads: ";
        for (uint32_t ch : clusterHeads)
        {
            std::cout << "Sensor-" << ch << " ";
        }
        std::cout << "\nNumber of Cluster Heads: " << clusterHeads.size() << "\n"
                  << "CH -> Base Station packets scheduled: " << clusterHeads.size() << "\n";

        if (round + 1 < m_nRounds)
        {
            Simulator::Schedule(Seconds(m_roundDuration),
                                &LeachRoundController::RunRound,
                                this,
                                round + 1);
        }
    }

    uint32_t m_nSensors;
    uint32_t m_nRounds;
    double m_roundDuration;
    double m_chProbability;
    uint16_t m_port;
    NodeContainer m_sensorNodes;
    Ipv4InterfaceContainer m_interfaces;
    EnergySourceContainer m_energySources;
    std::vector<bool> m_eligible;
    Ptr<UniformRandomVariable> m_randomVariable;
    AnimationInterface* m_anim;
    FlowMonitorHelper* m_flowHelper;
    Ptr<FlowMonitor> m_flowMonitor;
    std::string m_scenarioId;
    std::string m_stateCsv;
    std::string m_protocol;
    bool m_headerWritten;
    std::vector<uint64_t> m_previousTxBytes;
    std::vector<uint64_t> m_previousRxBytes;
    uint32_t m_sensorPacketsScheduled;
    uint32_t m_chPacketsScheduled;
};

static void
WriteCandidateMetrics(const std::string& path,
                      const std::string& scenarioId,
                      uint32_t seed,
                      uint64_t run,
                      uint32_t nSensors,
                      uint32_t rounds,
                      uint64_t txPackets,
                      uint64_t rxPackets,
                      uint64_t txBytes,
                      uint64_t rxBytes,
                      double pdrPercent,
                      double throughputMbps,
                      double averageDelayMs,
                      double initialEnergy,
                      double remainingEnergy,
                      double consumedEnergy,
                      uint64_t chTxPackets,
                      uint64_t chRxPackets)
{
    std::ofstream out(path, std::ios::trunc);
    if (!out.is_open())
        NS_FATAL_ERROR("Cannot open metrics output: " << path);

    out << std::setprecision(12);
    out << "{\n"
        << "  \"protocol\": \"" << g_protocol << "\",\n"
        << "  \"scenario_id\": \"" << scenarioId << "\",\n"
        << "  \"seed\": " << seed << ",\n"
        << "  \"run\": " << run << ",\n"
        << "  \"n_sensors\": " << nSensors << ",\n"
        << "  \"rounds\": " << rounds << ",\n"
        << "  \"forced_ch_mode\": " << (g_forceCH ? "true" : "false") << ",\n"
        << "  \"forced_ch_ids\": [";
    for (size_t i = 0; i < g_forcedCHIds.size(); ++i)
    {
        if (i) out << ", ";
        out << g_forcedCHIds[i];
    }
    out << "],\n"
        << "  \"packets_sent\": " << txPackets << ",\n"
        << "  \"packets_received\": " << rxPackets << ",\n"
        << "  \"tx_bytes\": " << txBytes << ",\n"
        << "  \"rx_bytes\": " << rxBytes << ",\n"
        << "  \"pdr\": " << (pdrPercent / 100.0) << ",\n"
        << "  \"pdr_percent\": " << pdrPercent << ",\n"
        << "  \"throughput_mbps\": " << throughputMbps << ",\n"
        << "  \"mean_delay_s\": " << (averageDelayMs / 1000.0) << ",\n"
        << "  \"mean_delay_ms\": " << averageDelayMs << ",\n"
        << "  \"packet_loss_ratio\": "
        << (txPackets ? 1.0 - static_cast<double>(rxPackets) / txPackets : 0.0) << ",\n"
        << "  \"energy_consumed_j\": " << consumedEnergy << ",\n"
        << "  \"remaining_energy_j\": " << remainingEnergy << ",\n"
        << "  \"initial_energy_j\": " << initialEnergy << ",\n"
        << "  \"ch_to_bs_tx_packets\": " << chTxPackets << ",\n"
        << "  \"ch_to_bs_rx_packets\": " << chRxPackets << ",\n"
        << "  \"ch_to_bs_pdr\": "
        << (chTxPackets ? static_cast<double>(chRxPackets) / chTxPackets : 0.0) << "\n"
        << "}\n";
}

int main(int argc, char* argv[])
{
    uint32_t nSensors = 100;
    double chProbability = 0.05;
    uint32_t numberOfRounds = 10;
    double roundDuration = 10.0;
    uint16_t port = 9000;
    uint32_t seed = 1;
    uint64_t run = 1;
    std::string scenarioId = "scenario_0";
    std::string protocol = "LEACH";
    std::string stateCsv = "node_states.csv";
    bool truncateStateCsv = true;
    std::string forcedCHIdsText = "";
    bool forceCH = false;
    std::string metricsOut = "candidate_metrics.json";

    CommandLine cmd(__FILE__);
    cmd.AddValue("nSensors", "Number of sensor nodes", nSensors);
    cmd.AddValue("chProbability", "LEACH CH probability", chProbability);
    cmd.AddValue("rounds", "Number of simulation rounds", numberOfRounds);
    cmd.AddValue("roundDuration", "Duration of one round in seconds", roundDuration);
    cmd.AddValue("port", "UDP port", port);
    cmd.AddValue("seed", "NS-3 RNG seed", seed);
    cmd.AddValue("run", "NS-3 RNG run number", run);
    cmd.AddValue("scenarioId", "Scenario identifier written to CSV", scenarioId);
    cmd.AddValue("protocol", "Protocol: Direct, LEACH, or C-LEACH", protocol);
    cmd.AddValue("stateCsv", "Per-node state CSV path", stateCsv);
    cmd.AddValue("truncateStateCsv", "Truncate state CSV before this run", truncateStateCsv);
    cmd.AddValue("forceCH", "Use a fixed candidate cluster-head set", forceCH);
    cmd.AddValue("forcedCHIds", "Comma-separated forced CH node IDs", forcedCHIdsText);
    cmd.AddValue("metricsOut", "Path for candidate metrics JSON", metricsOut);
    cmd.Parse(argc, argv);

    if (protocol != "Direct" && protocol != "LEACH" && protocol != "C-LEACH")
        NS_FATAL_ERROR("protocol must be Direct, LEACH, or C-LEACH");
    if (forceCH && protocol != "LEACH")
        NS_FATAL_ERROR("forceCH is supported only for LEACH mode");
    g_protocol = protocol;

    g_forceCH = forceCH;
    if (g_forceCH)
    {
        g_forcedCHIds = ParseForcedCHIds(forcedCHIdsText);
        if (g_forcedCHIds.size() > nSensors)
            NS_FATAL_ERROR("Forced CH count exceeds sensor count");
        for (uint32_t id : g_forcedCHIds)
            if (id >= nSensors) NS_FATAL_ERROR("Forced CH ID out of range: " << id);
    }

    if (nSensors < 2 || chProbability <= 0.0 || chProbability > 1.0 ||
        numberOfRounds == 0 || roundDuration <= 3.5)
    {
        NS_FATAL_ERROR("Invalid parameters. Require nSensors>=2, p in (0,1], rounds>0, roundDuration>3.5.");
    }

    RngSeedManager::SetSeed(seed);
    RngSeedManager::SetRun(run);

    if (truncateStateCsv)
    {
        std::ofstream clear(stateCsv, std::ios::trunc);
        if (!clear.is_open())
        {
            NS_FATAL_ERROR("Cannot create state CSV: " << stateCsv);
        }
    }

    Ptr<UniformRandomVariable> randomVariable = CreateObject<UniformRandomVariable>();
    std::vector<bool> eligible(nSensors, true);

    NodeContainer sensorNodes;
    sensorNodes.Create(nSensors);
    g_sensorNodesForElection = new NodeContainer(sensorNodes);
    NodeContainer baseStation;
    baseStation.Create(1);

    NodeContainer allNodes;
    allNodes.Add(sensorNodes);
    allNodes.Add(baseStation);

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211b);

    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");

    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    wifiPhy.SetChannel(wifiChannel.Create());

    NetDeviceContainer sensorDevices = wifi.Install(wifiPhy, wifiMac, sensorNodes);
    NetDeviceContainer bsDevice = wifi.Install(wifiPhy, wifiMac, baseStation);

    InternetStackHelper internet;
    internet.Install(allNodes);

    NetDeviceContainer allDevices;
    allDevices.Add(sensorDevices);
    allDevices.Add(bsDevice);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.0.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = ipv4.Assign(allDevices);
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // Energy source and radio model.
    BasicEnergySourceHelper energySourceHelper;
    energySourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(100.0));
    EnergySourceContainer sensorEnergySources = energySourceHelper.Install(sensorNodes);
    g_energyForElection = new EnergySourceContainer(sensorEnergySources);

    WifiRadioEnergyModelHelper radioEnergyHelper;
    radioEnergyHelper.Install(sensorDevices, sensorEnergySources);

    // Mobility is installed before the round controller is started.
    MobilityHelper mobility;
    mobility.SetPositionAllocator(
        "ns3::RandomRectanglePositionAllocator",
        "X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=100.0]"),
        "Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=100.0]"));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(sensorNodes);

    Ptr<ListPositionAllocator> bsPosition = CreateObject<ListPositionAllocator>();
    bsPosition->Add(Vector(50.0, 120.0, 0.0));
    mobility.SetPositionAllocator(bsPosition);
    mobility.Install(baseStation);

    // UDP sinks are started before the round events. FlowMonitor is installed
    // before Simulator::Run(), not after traffic events have been scheduled.
    PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApps = sinkHelper.Install(allNodes);
    sinkApps.Start(Seconds(0.0));
    sinkApps.Stop(Seconds(numberOfRounds * roundDuration));

    AnimationInterface anim("leach-performance.xml");
    for (uint32_t i = 0; i < nSensors; ++i)
    {
        anim.UpdateNodeDescription(sensorNodes.Get(i), "Sensor-" + std::to_string(i));
        anim.UpdateNodeColor(sensorNodes.Get(i), 0, 0, 255);
    }
    anim.UpdateNodeDescription(baseStation.Get(0), "Base-Station");
    anim.UpdateNodeColor(baseStation.Get(0), 255, 0, 0);

    FlowMonitorHelper flowHelper;
    Ptr<FlowMonitor> flowMonitor = flowHelper.Install(allNodes);

    LeachRoundController controller(nSensors,
                                    numberOfRounds,
                                    roundDuration,
                                    chProbability,
                                    port,
                                    sensorNodes,
                                    interfaces,
                                    sensorEnergySources,
                                    eligible,
                                    randomVariable,
                                    &anim,
                                    &flowHelper,
                                    flowMonitor,
                                    scenarioId,
                                    stateCsv,
                                    protocol);
    controller.Start();

    Simulator::Stop(Seconds(numberOfRounds * roundDuration));
    Simulator::Run();

    flowMonitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowHelper.GetClassifier());
    const auto stats = flowMonitor->GetFlowStats();

    uint64_t txPackets = 0, rxPackets = 0, txBytes = 0, rxBytes = 0;
    uint64_t chTxPackets = 0, chRxPackets = 0;
    Time totalDelay = Seconds(0);
    const Ipv4Address bsAddress = interfaces.GetAddress(nSensors);

    for (const auto& item : stats)
    {
        const auto tuple = classifier->FindFlow(item.first);
        const auto& fs = item.second;
        txPackets += fs.txPackets;
        rxPackets += fs.rxPackets;
        txBytes += fs.txBytes;
        rxBytes += fs.rxBytes;
        totalDelay += fs.delaySum;

        if (tuple.destinationAddress == bsAddress)
        {
            chTxPackets += fs.txPackets;
            chRxPackets += fs.rxPackets;
        }
    }

    const double pdr = txPackets ? 100.0 * rxPackets / txPackets : 0.0;
    const double throughputMbps =
        (numberOfRounds * roundDuration > 0.0)
            ? (rxBytes * 8.0) / (numberOfRounds * roundDuration) / 1e6
            : 0.0;
    const double averageDelayMs =
        rxPackets ? 1000.0 * totalDelay.GetSeconds() / rxPackets : 0.0;

    double totalRemainingEnergy = 0.0;
    for (uint32_t i = 0; i < sensorEnergySources.GetN(); ++i)
    {
        Ptr<BasicEnergySource> source =
            DynamicCast<BasicEnergySource>(sensorEnergySources.Get(i));
        if (source)
        {
            totalRemainingEnergy += source->GetRemainingEnergy();
        }
    }
    const double initialEnergy = nSensors * 100.0;
    const double consumedEnergy = initialEnergy - totalRemainingEnergy;

    std::cout << "\n--- PDR ---\n"
              << "Overall PDR: " << pdr << " %\n"
              << "CH -> BS PDR: "
              << (chTxPackets ? 100.0 * chRxPackets / chTxPackets : 0.0) << " %\n"
              << "\n--- THROUGHPUT ---\n"
              << "Overall Throughput: " << throughputMbps << " Mbps\n"
              << "\n--- DELAY ---\n"
              << "Overall Average Delay: " << averageDelayMs << " ms\n"
              << "\n--- ENERGY ---\n"
              << "Total Initial Energy: " << initialEnergy << " J\n"
              << "Total Remaining Energy: " << totalRemainingEnergy << " J\n"
              << "Total Energy Consumed: " << consumedEnergy << " J\n";

    // Save a compact baseline summary. These are simulation outputs, not training labels.
    std::ofstream performance("leach-performance.csv", std::ios::trunc);
    performance << "protocol,scenario_id,seed,run,rounds,n_sensors,tx_packets,rx_packets,tx_bytes,rx_bytes,"
                   "pdr_percent,throughput_mbps,average_delay_ms,initial_energy_j,"
                   "remaining_energy_j,energy_consumed_j,ch_to_bs_tx_packets,ch_to_bs_rx_packets\n";
    performance << protocol << "," << scenarioId << "," << seed << "," << run << "," << numberOfRounds << ","
                << nSensors << "," << txPackets << "," << rxPackets << "," << txBytes << ","
                << rxBytes << "," << pdr << "," << throughputMbps << "," << averageDelayMs << ","
                << initialEnergy << "," << totalRemainingEnergy << "," << consumedEnergy << ","
                << chTxPackets << "," << chRxPackets << "\n";
    performance.close();

    WriteCandidateMetrics(metricsOut, scenarioId, seed, run, nSensors,
                          numberOfRounds, txPackets, rxPackets, txBytes, rxBytes,
                          pdr, throughputMbps, averageDelayMs, initialEnergy,
                          totalRemainingEnergy, consumedEnergy, chTxPackets, chRxPackets);

    flowMonitor->SerializeToXmlFile("leach-flowmon.xml", true, true);
    std::cout << "Node-state data saved to: " << stateCsv << "\n"
              << "Performance data saved to: leach-performance.csv\n"
              << "FlowMonitor data saved to: leach-flowmon.xml\n";

    Simulator::Destroy();
    return 0;
}
