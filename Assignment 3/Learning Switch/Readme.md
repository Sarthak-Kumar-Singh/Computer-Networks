# Learning Switch

## Overview

The **Learning Switch** is an improvement over the Hub Controller, as it learns the MAC-to-port mappings from the incoming traffic. It installs flow rules on the switches based on these mappings, allowing direct communication between hosts without unnecessary broadcast traffic.

## Features

- Dynamically learns the MAC addresses of hosts and their corresponding ports.
- Installs flow rules for direct communication, avoiding broadcasting unnecessary traffic.
- More efficient and scalable than the Hub Controller.

## How It Works

1. When a packet arrives at a switch, the switch checks its MAC address table.
2. If the destination MAC address is already known, the switch forwards the packet directly to the appropriate port.
3. If the MAC address is unknown, the switch broadcasts the packet, and the controller learns the mapping.
4. The controller installs flow rules for direct forwarding as it learns more about the network.

## How to Run

1. Clone this repository:

    ```bash
    git clone <repository-url>
    cd learning_switch
    ```

2. Start the Mininet topology:

    ```bash
    sudo mn --custom topology.py --topo mytopo --controller remote
    ```

3. Run the Ryu Learning Switch Controller:

    ```bash
    ryu-manager learning_switch.py
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

- **Ping Test:** Initial packets may be broadcast until the controller learns the MAC-to-port mappings. Once learned, packets are forwarded directly.
- **Throughput Test:** The throughput is higher compared to the Hub Controller because unnecessary traffic is minimized, leading to less congestion.

## Conclusion

The Learning Switch is more efficient and scalable than the Hub Controller. By learning the MAC-to-port mappings, it reduces broadcast traffic and significantly improves network throughput.

---

## Additional Notes

- Implementations use the Mininet network emulator and the Ryu SDN controller framework.
- The given topology file (\`topology.py\`) sets up a network with multiple hosts and switches for testing purposes.

## References

- [Mininet Documentation](http://mininet.org)
- [Ryu SDN Controller](https://osrg.github.io/ryu/)

---
