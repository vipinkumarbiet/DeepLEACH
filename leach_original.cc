#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/energy-module.h"
#include "ns3/netanim-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"

#include <cmath>
#include <limits>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

using namespace ns3;
using namespace ns3::energy;

NS_LOG_COMPONENT_DEFINE("LeachPerformance");


// ============================================================
// LEACH CLUSTER HEAD SELECTION
// ============================================================

std::vector<uint32_t>
SelectClusterHeads(
    uint32_t round,
    uint32_t nSensors,
    double probability,
    std::vector<bool>& eligible,
    Ptr<UniformRandomVariable> randomVariable)
{
    std::vector<uint32_t> clusterHeads;

    uint32_t epochLength =
        static_cast<uint32_t>(1.0 / probability);

    // Reset eligibility at beginning of new epoch
    if (round > 0 &&
        round % epochLength == 0)
    {
        for (uint32_t i = 0;
             i < nSensors;
             i++)
        {
            eligible[i] = true;
        }

        std::cout
            << "\n*** Starting new LEACH epoch ***\n";
    }

    // LEACH threshold
    double denominator =
        1.0 -
        probability *
            (round % epochLength);

    double threshold =
        probability / denominator;

    // Select CHs
    for (uint32_t i = 0;
         i < nSensors;
         i++)
    {
        if (!eligible[i])
        {
            continue;
        }

        double randomNumber =
            randomVariable->GetValue(
                0.0,
                1.0);

        if (randomNumber < threshold)
        {
            clusterHeads.push_back(i);

            eligible[i] = false;
        }
    }

    return clusterHeads;
}


// ============================================================
// ASSIGN SENSOR TO NEAREST CLUSTER HEAD
// ============================================================

std::vector<int>
AssignClusters(
    NodeContainer& sensorNodes,
    const std::vector<uint32_t>& clusterHeads)
{
    std::vector<int> clusterAssignment(
        sensorNodes.GetN(),
        -1);

    if (clusterHeads.empty())
    {
        return clusterAssignment;
    }

    for (uint32_t i = 0;
         i < sensorNodes.GetN();
         i++)
    {
        // ----------------------------------------------------
        // Check whether node is CH
        // ----------------------------------------------------

        bool isClusterHead = false;

        for (uint32_t ch :
             clusterHeads)
        {
            if (i == ch)
            {
                isClusterHead = true;

                clusterAssignment[i] =
                    static_cast<int>(ch);

                break;
            }
        }

        if (isClusterHead)
        {
            continue;
        }


        // ----------------------------------------------------
        // Sensor position
        // ----------------------------------------------------

        Ptr<MobilityModel>
            sensorMobility =
                sensorNodes.Get(i)
                    ->GetObject<MobilityModel>();

        Vector sensorPosition =
            sensorMobility->GetPosition();


        double minimumDistance =
            std::numeric_limits<double>::max();

        int nearestCH = -1;


        // ----------------------------------------------------
        // Find nearest CH
        // ----------------------------------------------------

        for (uint32_t ch :
             clusterHeads)
        {
            Ptr<MobilityModel>
                chMobility =
                    sensorNodes.Get(ch)
                        ->GetObject<MobilityModel>();

            Vector chPosition =
                chMobility->GetPosition();


            double dx =
                sensorPosition.x -
                chPosition.x;

            double dy =
                sensorPosition.y -
                chPosition.y;


            double distance =
                std::sqrt(
                    dx * dx +
                    dy * dy);


            if (distance <
                minimumDistance)
            {
                minimumDistance =
                    distance;

                nearestCH =
                    static_cast<int>(ch);
            }
        }

        clusterAssignment[i] =
            nearestCH;
    }

    return clusterAssignment;
}


// ============================================================
// UPDATE NETANIM COLORS
// ============================================================

void
UpdateClusterHeadColors(
    AnimationInterface* anim,
    NodeContainer sensorNodes,
    std::vector<uint32_t> clusterHeads)
{
    // All sensors BLUE
    for (uint32_t i = 0;
         i < sensorNodes.GetN();
         i++)
    {
        anim->UpdateNodeColor(
            sensorNodes.Get(i),
            0,
            0,
            255);

        anim->UpdateNodeDescription(
            sensorNodes.Get(i),
            "Sensor-" +
                std::to_string(i));
    }


    // Cluster Heads GREEN
    for (uint32_t ch :
         clusterHeads)
    {
        anim->UpdateNodeColor(
            sensorNodes.Get(ch),
            0,
            255,
            0);

        anim->UpdateNodeDescription(
            sensorNodes.Get(ch),
            "CH-" +
                std::to_string(ch));
    }
}


// ============================================================
// MAIN
// ============================================================

int
main(int argc, char* argv[])
{
    // ========================================================
    // 1. SIMULATION PARAMETERS
    // ========================================================

    uint32_t nSensors = 100;

    double chProbability = 0.05;

    uint32_t numberOfRounds = 10;

    double roundDuration = 10.0;

    uint16_t port = 9000;


    // ========================================================
    // PACKET COUNTERS
    // ========================================================

    uint32_t sensorPacketsScheduled = 0;

    uint32_t chPacketsScheduled = 0;


    // ========================================================
    // COMMAND LINE
    // ========================================================

    CommandLine cmd;

    cmd.AddValue(
        "nSensors",
        "Number of sensor nodes",
        nSensors);

    cmd.Parse(
        argc,
        argv);


    // ========================================================
    // RANDOM VARIABLE
    // ========================================================

    Ptr<UniformRandomVariable>
        randomVariable =
            CreateObject<
                UniformRandomVariable>();


    std::vector<bool> eligible(
        nSensors,
        true);


    // ========================================================
    // 2. CREATE SENSOR NODES
    // ========================================================

    NodeContainer sensorNodes;

    sensorNodes.Create(
        nSensors);


    // ========================================================
    // 3. CREATE BASE STATION
    // ========================================================

    NodeContainer baseStation;

    baseStation.Create(1);


    // All nodes
    NodeContainer allNodes;

    allNodes.Add(
        sensorNodes);

    allNodes.Add(
        baseStation);


    // ========================================================
    // 4. WI-FI
    // ========================================================

    WifiHelper wifi;

    wifi.SetStandard(
        WIFI_STANDARD_80211b);


    WifiMacHelper wifiMac;

    wifiMac.SetType(
        "ns3::AdhocWifiMac");


    YansWifiPhyHelper wifiPhy;

    YansWifiChannelHelper wifiChannel =
        YansWifiChannelHelper::Default();


    wifiPhy.SetChannel(
        wifiChannel.Create());


    // Sensor Wi-Fi devices
    NetDeviceContainer sensorDevices =
        wifi.Install(
            wifiPhy,
            wifiMac,
            sensorNodes);


    // Base Station Wi-Fi device
    NetDeviceContainer bsDevice =
        wifi.Install(
            wifiPhy,
            wifiMac,
            baseStation);


    // ========================================================
    // 5. INTERNET STACK
    // ========================================================

    InternetStackHelper internet;

    internet.Install(
        allNodes);


    // ========================================================
    // 6. IP ADDRESSING
    // ========================================================

    NetDeviceContainer allDevices;

    allDevices.Add(
        sensorDevices);

    allDevices.Add(
        bsDevice);


    Ipv4AddressHelper ipv4;

    ipv4.SetBase(
        "10.1.0.0",
        "255.255.255.0");


    Ipv4InterfaceContainer interfaces =
        ipv4.Assign(
            allDevices);


    // Routing
    Ipv4GlobalRoutingHelper::
        PopulateRoutingTables();


    // ========================================================
    // 7. UDP PACKET SINK
    // ========================================================

    PacketSinkHelper sinkHelper(
        "ns3::UdpSocketFactory",
        InetSocketAddress(
            Ipv4Address::GetAny(),
            port));


    // Install sink on all nodes
    ApplicationContainer sinkApps =
        sinkHelper.Install(
            allNodes);


    sinkApps.Start(
        Seconds(0.0));


    sinkApps.Stop(
        Seconds(
            numberOfRounds *
            roundDuration));


    // ========================================================
    // 8. ENERGY MODEL
    // ========================================================

    BasicEnergySourceHelper
        energySourceHelper;


    energySourceHelper.Set(
        "BasicEnergySourceInitialEnergyJ",
        DoubleValue(100.0));


    EnergySourceContainer
        sensorEnergySources =
            energySourceHelper.Install(
                sensorNodes);


    // Wi-Fi radio energy model
    WifiRadioEnergyModelHelper
        radioEnergyHelper;


    DeviceEnergyModelContainer
        sensorRadioEnergyModels =
            radioEnergyHelper.Install(
                sensorDevices,
                sensorEnergySources);


    // ========================================================
    // 9. SENSOR POSITIONS
    // ========================================================

    MobilityHelper mobility;


    mobility.SetPositionAllocator(
        "ns3::RandomRectanglePositionAllocator",

        "X",
        StringValue(
            "ns3::UniformRandomVariable"
            "[Min=0.0|Max=100.0]"),

        "Y",
        StringValue(
            "ns3::UniformRandomVariable"
            "[Min=0.0|Max=100.0]"));


    mobility.SetMobilityModel(
        "ns3::ConstantPositionMobilityModel");


    mobility.Install(
        sensorNodes);


    // ========================================================
    // 10. BASE STATION POSITION
    // ========================================================

    Ptr<ListPositionAllocator>
        bsPosition =
            CreateObject<
                ListPositionAllocator>();


    bsPosition->Add(
        Vector(
            50.0,
            120.0,
            0.0));


    mobility.SetPositionAllocator(
        bsPosition);


    mobility.Install(
        baseStation);


    // ========================================================
    // 11. NETANIM
    // ========================================================

    AnimationInterface anim(
        "leach-performance.xml");


    // Sensor nodes
    for (uint32_t i = 0;
         i < nSensors;
         i++)
    {
        anim.UpdateNodeDescription(
            sensorNodes.Get(i),
            "Sensor-" +
                std::to_string(i));


        anim.UpdateNodeColor(
            sensorNodes.Get(i),
            0,
            0,
            255);
    }


    // Base Station
    anim.UpdateNodeDescription(
        baseStation.Get(0),
        "Base-Station");


    anim.UpdateNodeColor(
        baseStation.Get(0),
        255,
        0,
        0);


    // ========================================================
    // 12. LEACH ROUNDS
    // ========================================================

    for (uint32_t round = 0;
         round < numberOfRounds;
         round++)
    {
        // ====================================================
        // STEP 12A: SELECT CHs
        // ====================================================

        std::vector<uint32_t>
            clusterHeads =
                SelectClusterHeads(
                    round,
                    nSensors,
                    chProbability,
                    eligible,
                    randomVariable);


        // ====================================================
        // STEP 12B: CLUSTER ASSIGNMENT
        // ====================================================

        std::vector<int>
            clusterAssignment =
                AssignClusters(
                    sensorNodes,
                    clusterHeads);


        // ====================================================
        // STEP 12C: SENSOR -> CH
        // ====================================================

        for (uint32_t i = 0;
             i < nSensors;
             i++)
        {
            // CH does not send to itself
            if (clusterAssignment[i] ==
                static_cast<int>(i))
            {
                continue;
            }


            // No CH
            if (clusterAssignment[i] < 0)
            {
                continue;
            }


            uint32_t ch =
                static_cast<uint32_t>(
                    clusterAssignment[i]);


            // CH IP
            Ipv4Address chAddress =
                interfaces.GetAddress(
                    ch);


            // UDP client
            UdpClientHelper client(
                chAddress,
                port);


            client.SetAttribute(
                "MaxPackets",
                UintegerValue(1));


            client.SetAttribute(
                "Interval",
                TimeValue(
                    Seconds(1.0)));


            client.SetAttribute(
                "PacketSize",
                UintegerValue(100));


            ApplicationContainer
                clientApp =
                    client.Install(
                        sensorNodes.Get(i));


            // Sensor -> CH at +1 second
            double sendTime =
                round *
                    roundDuration +
                1.0;


            clientApp.Start(
                Seconds(sendTime));


            clientApp.Stop(
                Seconds(
                    sendTime + 0.5));


            sensorPacketsScheduled++;
        }


        // ====================================================
        // STEP 12D: CH -> BASE STATION
        // ====================================================

        // Base station interface index = nSensors
        Ipv4Address bsAddress =
            interfaces.GetAddress(
                nSensors);


        for (uint32_t ch :
             clusterHeads)
        {
            UdpClientHelper chClient(
                bsAddress,
                port);


            chClient.SetAttribute(
                "MaxPackets",
                UintegerValue(1));


            chClient.SetAttribute(
                "Interval",
                TimeValue(
                    Seconds(1.0)));


            chClient.SetAttribute(
                "PacketSize",
                UintegerValue(100));


            ApplicationContainer chApp =
                chClient.Install(
                    sensorNodes.Get(ch));


            // CH -> BS at +3 seconds
            double chSendTime =
                round *
                    roundDuration +
                3.0;


            chApp.Start(
                Seconds(chSendTime));


            chApp.Stop(
                Seconds(
                    chSendTime + 0.5));


            chPacketsScheduled++;
        }


        // ====================================================
        // STEP 12E: NETANIM
        // ====================================================

        Simulator::Schedule(
            Seconds(
                round *
                roundDuration),

            &UpdateClusterHeadColors,

            &anim,

            sensorNodes,

            clusterHeads);


        // ====================================================
        // PRINT ROUND INFORMATION
        // ====================================================

        std::cout
            << "\n====================================\n";

        std::cout
            << "LEACH Round: "
            << round + 1
            << "\n";

        std::cout
            << "====================================\n";


        uint32_t epochLength =
            static_cast<uint32_t>(
                1.0 / chProbability);


        double threshold =
            chProbability /
            (
                1.0 -
                chProbability *
                    (round %
                     epochLength)
            );


        std::cout
            << "Threshold: "
            << threshold
            << "\n";


        std::cout
            << "Cluster Heads: ";


        for (uint32_t ch :
             clusterHeads)
        {
            std::cout
                << "Sensor-"
                << ch
                << " ";
        }


        std::cout
            << "\n";


        std::cout
            << "Number of Cluster Heads: "
            << clusterHeads.size()
            << "\n";


        std::cout
            << "CH -> Base Station packets: "
            << clusterHeads.size()
            << "\n";
    }


    // ========================================================
    // 13. FLOW MONITOR
    // ========================================================

    FlowMonitorHelper flowHelper;

    Ptr<FlowMonitor> flowMonitor =
        flowHelper.Install(
            allNodes);


    // ========================================================
    // 14. STOP SIMULATION
    // ========================================================

    Simulator::Stop(
        Seconds(
            numberOfRounds *
            roundDuration));


    // ========================================================
    // 15. RUN
    // ========================================================

    Simulator::Run();


    // ========================================================
    // 16. FLOW MONITOR RESULTS
    // ========================================================

    flowMonitor->
        CheckForLostPackets();


    Ptr<Ipv4FlowClassifier>
        classifier =
            DynamicCast<
                Ipv4FlowClassifier>(
                    flowHelper.GetClassifier());


    std::map<
        FlowId,
        FlowMonitor::FlowStats>
        stats =
            flowMonitor->GetFlowStats();


    // --------------------------------------------------------
    // Overall statistics
    // --------------------------------------------------------

    uint64_t totalTxPackets = 0;

    uint64_t totalRxPackets = 0;

    uint64_t totalTxBytes = 0;

    uint64_t totalRxBytes = 0;

    Time totalDelay =
        Seconds(0);


    // --------------------------------------------------------
    // Sensor -> CH statistics
    // --------------------------------------------------------

    uint64_t sensorTxPackets = 0;

    uint64_t sensorRxPackets = 0;

    uint64_t sensorTxBytes = 0;

    uint64_t sensorRxBytes = 0;

    Time sensorDelay =
        Seconds(0);


    // --------------------------------------------------------
    // CH -> BS statistics
    // --------------------------------------------------------

    uint64_t chTxPackets = 0;

    uint64_t chRxPackets = 0;

    uint64_t chTxBytes = 0;

    uint64_t chRxBytes = 0;

    Time chDelay =
        Seconds(0);


    // Base station IP
    Ipv4Address bsAddress =
        interfaces.GetAddress(
            nSensors);


    // ========================================================
    // PROCESS EVERY FLOW
    // ========================================================

    for (auto const& flow :
         stats)
    {
        FlowId flowId =
            flow.first;


        FlowMonitor::FlowStats
            flowStats =
                flow.second;


        Ipv4FlowClassifier::FiveTuple
            tuple =
                classifier->FindFlow(
                    flowId);


        // Overall
        totalTxPackets +=
            flowStats.txPackets;

        totalRxPackets +=
            flowStats.rxPackets;

        totalTxBytes +=
            flowStats.txBytes;

        totalRxBytes +=
            flowStats.rxBytes;

        totalDelay +=
            flowStats.delaySum;


        // ----------------------------------------------------
        // CH -> Base Station
        // ----------------------------------------------------

        if (tuple.destinationAddress ==
            bsAddress)
        {
            chTxPackets +=
                flowStats.txPackets;

            chRxPackets +=
                flowStats.rxPackets;

            chTxBytes +=
                flowStats.txBytes;

            chRxBytes +=
                flowStats.rxBytes;

            chDelay +=
                flowStats.delaySum;
        }


        // ----------------------------------------------------
        // Sensor -> CH
        // ----------------------------------------------------

        else
        {
            sensorTxPackets +=
                flowStats.txPackets;

            sensorRxPackets +=
                flowStats.rxPackets;

            sensorTxBytes +=
                flowStats.txBytes;

            sensorRxBytes +=
                flowStats.rxBytes;

            sensorDelay +=
                flowStats.delaySum;
        }
    }


    // ========================================================
    // 17. CALCULATE PERFORMANCE METRICS
    // ========================================================

    double totalSimulationTime =
        numberOfRounds *
        roundDuration;


    // --------------------------------------------------------
    // Overall PDR
    // --------------------------------------------------------

    double overallPdr = 0.0;

    if (totalTxPackets > 0)
    {
        overallPdr =
            100.0 *
            static_cast<double>(
                totalRxPackets) /
            static_cast<double>(
                totalTxPackets);
    }


    // --------------------------------------------------------
    // Sensor -> CH PDR
    // --------------------------------------------------------

    double sensorPdr = 0.0;

    if (sensorTxPackets > 0)
    {
        sensorPdr =
            100.0 *
            static_cast<double>(
                sensorRxPackets) /
            static_cast<double>(
                sensorTxPackets);
    }


    // --------------------------------------------------------
    // CH -> BS PDR
    // --------------------------------------------------------

    double chPdr = 0.0;

    if (chTxPackets > 0)
    {
        chPdr =
            100.0 *
            static_cast<double>(
                chRxPackets) /
            static_cast<double>(
                chTxPackets);
    }


    // --------------------------------------------------------
    // Packet loss
    // --------------------------------------------------------

    uint64_t totalPacketLoss =
        totalTxPackets -
        totalRxPackets;


    // --------------------------------------------------------
    // Throughput
    // --------------------------------------------------------

    double throughputMbps =
        (
            static_cast<double>(
                totalRxBytes) *
            8.0
        )
        /
        (
            totalSimulationTime *
            1000000.0
        );


    // Sensor -> CH throughput
    double sensorThroughputMbps =
        (
            static_cast<double>(
                sensorRxBytes) *
            8.0
        )
        /
        (
            totalSimulationTime *
            1000000.0
        );


    // CH -> BS throughput
    double chThroughputMbps =
        (
            static_cast<double>(
                chRxBytes) *
            8.0
        )
        /
        (
            totalSimulationTime *
            1000000.0
        );


    // --------------------------------------------------------
    // Average delay
    // --------------------------------------------------------

    double averageDelayMs = 0.0;

    if (totalRxPackets > 0)
    {
        averageDelayMs =
            totalDelay.GetSeconds() *
            1000.0 /
            static_cast<double>(
                totalRxPackets);
    }


    double sensorAverageDelayMs = 0.0;

    if (sensorRxPackets > 0)
    {
        sensorAverageDelayMs =
            sensorDelay.GetSeconds() *
            1000.0 /
            static_cast<double>(
                sensorRxPackets);
    }


    double chAverageDelayMs = 0.0;

    if (chRxPackets > 0)
    {
        chAverageDelayMs =
            chDelay.GetSeconds() *
            1000.0 /
            static_cast<double>(
                chRxPackets);
    }


    // ========================================================
    // 18. ENERGY RESULTS
    // ========================================================

    double totalInitialEnergy = 0.0;

    double totalRemainingEnergy = 0.0;


    for (uint32_t i = 0;
         i < nSensors;
         i++)
    {
        Ptr<energy::BasicEnergySource>
            energySource =
                DynamicCast<
                    energy::BasicEnergySource>(
                        sensorEnergySources.Get(i));


        totalInitialEnergy +=
            energySource->
                GetInitialEnergy();


        totalRemainingEnergy +=
            energySource->
                GetRemainingEnergy();
    }


    double totalEnergyConsumed =
        totalInitialEnergy -
        totalRemainingEnergy;


    double averageRemainingEnergy =
        totalRemainingEnergy /
        static_cast<double>(
            nSensors);


    // ========================================================
    // 19. PRINT PERFORMANCE RESULTS
    // ========================================================

    std::cout
        << "\n\n====================================\n";

    std::cout
        << "LEACH PERFORMANCE RESULTS\n";

    std::cout
        << "====================================\n";


    std::cout
        << "\n--- PACKET STATISTICS ---\n";


    std::cout
        << "Scheduled Sensor -> CH: "
        << sensorPacketsScheduled
        << "\n";


    std::cout
        << "Scheduled CH -> BS: "
        << chPacketsScheduled
        << "\n";


    std::cout
        << "Actual transmitted packets: "
        << totalTxPackets
        << "\n";


    std::cout
        << "Actual received packets: "
        << totalRxPackets
        << "\n";


    std::cout
        << "Packet loss: "
        << totalPacketLoss
        << "\n";


    std::cout
        << "\n--- PDR ---\n";


    std::cout
        << "Overall PDR: "
        << overallPdr
        << " %\n";


    std::cout
        << "Sensor -> CH PDR: "
        << sensorPdr
        << " %\n";


    std::cout
        << "CH -> BS PDR: "
        << chPdr
        << " %\n";


    std::cout
        << "\n--- THROUGHPUT ---\n";


    std::cout
        << "Overall Throughput: "
        << throughputMbps
        << " Mbps\n";


    std::cout
        << "Sensor -> CH Throughput: "
        << sensorThroughputMbps
        << " Mbps\n";


    std::cout
        << "CH -> BS Throughput: "
        << chThroughputMbps
        << " Mbps\n";


    std::cout
        << "\n--- DELAY ---\n";


    std::cout
        << "Overall Average Delay: "
        << averageDelayMs
        << " ms\n";


    std::cout
        << "Sensor -> CH Average Delay: "
        << sensorAverageDelayMs
        << " ms\n";


    std::cout
        << "CH -> BS Average Delay: "
        << chAverageDelayMs
        << " ms\n";


    std::cout
        << "\n--- ENERGY ---\n";


    std::cout
        << "Total Initial Energy: "
        << totalInitialEnergy
        << " J\n";


    std::cout
        << "Total Remaining Energy: "
        << totalRemainingEnergy
        << " J\n";


    std::cout
        << "Total Energy Consumed: "
        << totalEnergyConsumed
        << " J\n";


    std::cout
        << "Average Remaining Energy per Sensor: "
        << averageRemainingEnergy
        << " J\n";


    // ========================================================
    // 20. SAVE PERFORMANCE CSV
    // ========================================================

    std::ofstream csv(
        "leach-performance.csv");


    if (csv.is_open())
    {
        csv
            << "Metric,Value\n";


        csv
            << "Scheduled Sensor-CH,"
            << sensorPacketsScheduled
            << "\n";


        csv
            << "Scheduled CH-BS,"
            << chPacketsScheduled
            << "\n";


        csv
            << "Actual TX Packets,"
            << totalTxPackets
            << "\n";


        csv
            << "Actual RX Packets,"
            << totalRxPackets
            << "\n";


        csv
            << "Packet Loss,"
            << totalPacketLoss
            << "\n";


        csv
            << "Overall PDR (%),"
            << overallPdr
            << "\n";


        csv
            << "Sensor-CH PDR (%),"
            << sensorPdr
            << "\n";


        csv
            << "CH-BS PDR (%),"
            << chPdr
            << "\n";


        csv
            << "Overall Throughput (Mbps),"
            << throughputMbps
            << "\n";


        csv
            << "Sensor-CH Throughput (Mbps),"
            << sensorThroughputMbps
            << "\n";


        csv
            << "CH-BS Throughput (Mbps),"
            << chThroughputMbps
            << "\n";


        csv
            << "Overall Delay (ms),"
            << averageDelayMs
            << "\n";


        csv
            << "Sensor-CH Delay (ms),"
            << sensorAverageDelayMs
            << "\n";


        csv
            << "CH-BS Delay (ms),"
            << chAverageDelayMs
            << "\n";


        csv
            << "Total Initial Energy (J),"
            << totalInitialEnergy
            << "\n";


        csv
            << "Total Remaining Energy (J),"
            << totalRemainingEnergy
            << "\n";


        csv
            << "Total Energy Consumed (J),"
            << totalEnergyConsumed
            << "\n";


        csv
            << "Average Remaining Energy (J),"
            << averageRemainingEnergy
            << "\n";


        csv.close();


        std::cout
            << "\nPerformance data saved to:"
            << " leach-performance.csv\n";
    }
    else
    {
        std::cout
            << "\nERROR: Could not create CSV file.\n";
    }


    // ========================================================
    // 21. SAVE FLOWMONITOR XML
    // ========================================================

    flowMonitor->
        SerializeToXmlFile(
            "leach-flowmon.xml",
            true,
            true);


    std::cout
        << "FlowMonitor data saved to:"
        << " leach-flowmon.xml\n";


    // ========================================================
    // 22. DESTROY
    // ========================================================

    Simulator::Destroy();


    return 0;
}