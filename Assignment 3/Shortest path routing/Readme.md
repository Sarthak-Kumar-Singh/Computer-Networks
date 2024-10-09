# Shortest Path Routing with Kruskal's Algorithm and LLDP

## Overview

This implementation enhances the traditional Layer 2 (L2) routing by incorporating **Shortest Path Routing** based on link delays and Spanning Tree Protocol (STP) to avoid loops. The controller performs dynamic link delay measurements using **LLDP (Link Layer Discovery Protocol)** packets and computes the shortest path using **Dijkstra’s Algorithm** based on real-time link conditions. The shortest paths are computed based on link delay as weights, optimizing packet routing.

## Approach

### 1. Spanning Tree Construction (Kruskal’s Algorithm)

The **Spanning Tree Protocol (STP)** is used to prevent loops in the network for multicast packet forwarding. We utilize **Kruskal's Algorithm** to construct the spanning tree and ensure that packets are forwarded only through the ports that are part of this tree. By preventing broadcast storms and loops, we ensure efficient multicast distribution across the network.

### 2. LLDP for Link Delay Measurement

To measure link delays, **custom LLDP packets** are used. These packets contain a unique identifier in the form of a counter value for each transmission.

- **LLDP Packets**: Upon detecting a new link, an LLDP packet is exchanged between the two switches. The time of transmission and reception is used to calculate the link delay.
- **Delay Calculation**: The delay is calculated as:
    ```
    Delay = End Time − Start Time
    ```
  The measured delays are stored for later use in the shortest path computation.

### 3. Topology Graph with Weights

Once the link delays are measured using LLDP, the topology graph is updated with these delay values as edge weights. This updated graph allows the controller to optimize routing decisions based on real-time network conditions.

### 4. Routing Table and Shortest Path Algorithm

To compute the optimal routes, **Dijkstra’s Algorithm** is applied. It calculates the shortest path between each switch (node) based on link delays, ensuring packets are forwarded along the most efficient paths.

- **Routing Table**: The routing table is constructed using the shortest paths computed by Dijkstra's algorithm. It maps each destination switch to the outgoing port from the current switch.

  The routing table is structured as follows:
    ```
    Routing Table[i][j] = Port from switch i to switch j
    ```
    where:
    - **i** and **j** are the switch identifiers (DPIDs).
    - **Port** represents the outgoing port from switch `i` to reach switch `j`.

- **MAC Address Mapping**: A mapping of **MAC addresses** to switch IDs (DPIDs) is maintained. This mapping allows efficient routing of packets to their destination based on the destination MAC address.

### 5. Dijkstra’s Algorithm for Optimal Path Calculation

The shortest paths between switches are calculated using **Dijkstra’s Algorithm**. Link delays are used as edge weights to optimize path selection. This dynamic approach ensures that packets are forwarded through the network with minimal delays, adjusting to real-time network conditions.

## Assumptions

- **LLDP packets** are assumed to travel without interference, and the measured delays are reflective of actual link performance.
- Switches respond accurately to LLDP packets, and there is no packet loss during transmission.
- Delay calculations assume minimal processing overhead at the switches.

## How to Run

1. Clone this repository:

    ```bash
    git clone <repository-url>
    cd shortest_path_routing_with_kruskal_lldp
    ```

2. Start the Mininet topology (using the provided topology file):

    ```bash
    sudo mn --custom topology.py --topo mytopo --controller remote
    ```

3. Run the Ryu Shortest Path Routing Controller with Kruskal's Algorithm and LLDP:

    ```bash
    ryu-manager shortest_path_routing.py
    ```

4. In the Mininet CLI, run a \`pingall\` test to check network connectivity:

    ```bash
    mininet> pingall
    ```

5. Test the throughput between hosts using \`iperf\`, observing how the shortest path routing is affected by real-time link delays.

## Observations

- **Ping Test**: All hosts should be able to communicate with minimal latency due to optimized path selection based on link delays.
- **Throughput Test**: The throughput between hosts is expected to be higher, as packets are routed along paths with the least delay.

## Conclusion

This implementation of **Shortest Path Routing** ensures efficient packet forwarding by dynamically measuring link delays and computing the optimal paths using Dijkstra’s Algorithm. The use of **Kruskal's Algorithm** for spanning tree construction and **LLDP packets** for real-time delay measurement provides a robust and adaptive solution for managing network traffic.

By combining these techniques, the system can respond to changes in network conditions, ensuring packets are always routed through the most efficient paths, reducing latency and improving overall network performance.

## Additional Notes

- The implementation is built using the Mininet network emulator and the Ryu SDN controller framework.
- The network topology file (\`topology.py\`) sets up a test environment with different link capacities to evaluate the performance of the shortest path routing algorithm.

## References

- [Mininet Documentation](http://mininet.org)
- [Ryu SDN Controller](https://osrg.github.io/ryu/)
- [LLDP Protocol](https://en.wikipedia.org/wiki/Link_Layer_Discovery_Protocol)

---
