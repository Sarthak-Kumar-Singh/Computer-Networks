# Spanning Tree Implementation

## Overview

In Layer 2 (L2) networks, having loops in the topology can cause broadcast storms and packets getting stuck in loops. This issue can be efficiently addressed by constructing a **Spanning Tree**, which ensures that no cycles exist in the network topology, allowing the learning switch to forward packets correctly.

In this implementation, the learning switch is enhanced to handle loops in the network by constructing a spanning tree. The controller handles broadcast packets by forwarding them only to the active links in the spanning tree, thereby preventing the broadcast traffic from looping indefinitely.

## Features

- Constructs a **spanning tree** for the given network topology.
- Assumes all link weights are equal (set to 1).
- Handles broadcast packets by forwarding them only through open ports in the spanning tree and directly to any attached hosts.
- Prevents broadcast loops, improving network stability.

## How It Works

1. The controller constructs a spanning tree based on the network topology using a spanning tree algorithm (any valid algorithm can be used). In this implementation, all link weights are assumed to be 1.
2. When a broadcast packet arrives at a switch:
   - The switch forwards the packet only to the open ports that are part of the spanning tree.
   - The packet is also forwarded to any hosts that are directly attached to the switch.
3. This method ensures that broadcast packets are not caught in any loops, eliminating the issue of flooding in cyclic topologies.

## Assumptions

- All link weights are set to 1 for constructing the spanning tree.
- The controller builds the spanning tree for the entire network when it is initialized.
- Broadcast traffic is limited only to the paths defined by the spanning tree to prevent loops.

## How to Run

1. Clone this repository:

    ```bash
    git clone <repository-url>
    cd spanning_tree
    ```

2. Start the Mininet topology (you can use the provided four-node cyclic topology file):

    ```bash
    sudo mn --custom topology.py --topo mytopo --controller remote
    ```

3. Run the Ryu Spanning Tree Controller:

    ```bash
    ryu-manager --observe-links spanning_tree.py
    ```

4. In the Mininet CLI, run a \`pingall\` test to check network connectivity and ensure no loops occur:

    ```bash
    mininet> pingall
    ```

5. Test the broadcast handling across the network, ensuring that no broadcast loops are formed.

## Observations

- **Ping Test:** All hosts should be able to communicate without packet loss, and no broadcast loops should occur.
- **Broadcast Packets:** Broadcast traffic is limited to open ports in the spanning tree, ensuring efficient delivery without looping.

## Approach

- The spanning tree is constructed once at the start based on the network topology. We used a basic spanning tree construction algorithm with all links weighted equally (set to 1).
- For broadcast traffic, the switch forwards the packets only along paths in the spanning tree and to any directly connected hosts.
- The implementation is designed to handle any general network topology, although the provided topology is a four-node cyclic network.

## Conclusion

The spanning tree implementation prevents loops in the network by limiting broadcast traffic to the paths in the spanning tree. This approach ensures that the learning switch does not encounter broadcast storms, leading to more stable and efficient network performance.

## Additional Notes

- The code uses the Mininet network emulator and the Ryu SDN controller framework to simulate the network.
- The topology file (\`topology.py\`) sets up a cyclic network for testing purposes, but the controller is designed to handle any topology.

## References

- [Mininet Documentation](http://mininet.org)
- [Ryu SDN Controller](https://osrg.github.io/ryu/)

---
