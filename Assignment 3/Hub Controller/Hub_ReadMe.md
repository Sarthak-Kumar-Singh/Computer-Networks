# Hub Controller

## Overview

The **Hub Controller** works by redirecting all the traffic it receives to itself and then forwarding it to all switch ports except the incoming port. Essentially, it mimics the behavior of a physical hub, broadcasting traffic across the entire network. This approach is not very efficient since all hosts receive all traffic, even if it’s not intended for them.

## Features

- Redirects all traffic to the controller.
- Forwards the received traffic to all ports except the incoming one (broadcasting behavior).
- Simple implementation but not suitable for large networks due to excessive broadcasting.

## How It Works

1. When a packet arrives at the switch, it sends the packet to the controller.
2. The controller then forwards the packet to all other ports, except the incoming one.
3. This behavior continues for all packets.

## How to Run

1. Clone this repository:

    ```bash
    git clone <repository-url>
    cd hub_controller
    ```

2. Start the Mininet topology:

    ```bash
    sudo mn --custom topology.py --topo mytopo --controller remote
    ```

3. Run the Ryu Hub Controller:

    ```bash
    ryu-manager hub_controller.py
    ```

4. In the Mininet CLI, run a \`pingall\` test to check network connectivity:

    ```bash
    mininet> pingall
    ```

5. To run a throughput test between \`h1\` and \`h5\` using \`iperf\`, run the following commands in Mininet:

    ```bash
    mininet> iperf h1 h5
    ```

## Observations

- **Ping Test:** Since all traffic is broadcast, all hosts can reach each other.
- **Throughput Test:** The throughput may be lower due to network congestion caused by the broadcast traffic.

## Conclusion

The Hub Controller is simple to implement but inefficient for larger networks as all hosts receive unnecessary traffic, resulting in lower throughput and increased network congestion.

---
