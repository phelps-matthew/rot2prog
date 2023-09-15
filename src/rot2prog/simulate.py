"""Simulates ROT2Prog controller TX/RX"""
import logging
from threading import Lock, Thread

import serial


class ROT2ProgSim:

    """Receives commands and sends responses to simulate the ROT2Prog controller."""

    _log = None

    _ser = None
    _retry = 5
    _keep_running = True

    _az = 0
    _el = 0
    _pulses_per_degree = 0

    def __init__(self, port, pulses_per_degree):
        """Creates object, opens serial connection, and starts daemon thread to run simulator..

        Args:
            port (str): Name of serial port to connect to.
            pulses_per_degree (int): Resolution of simulated ROT2Prog controller. Options are 1, 2, and 4.
        """
        self._log = logging.getLogger(__name__)

        # open serial port
        self._ser = serial.Serial(
            port=port,
            baudrate=115200,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=None,
            inter_byte_timeout=0.1,
        )  # inter_byte_timeout allows continued operation after a bad packet

        self._pulses_per_degree = int(pulses_per_degree)
        self._log.info("ROT2Prog simulation interface opened on " + str(self._ser.name))

        # start daemon thread to communicate on serial port
        Thread(target=self._run, daemon=True).start()

    def _run(self):
        """Receives command packets, parses them to update the state of the simulator, and sends response packets when necessary."""
        while self._keep_running:
            command_packet = list(self._ser.read(13))
            self._log.debug(
                "Command packet received: " + str(list(map(hex, list(command_packet))))
            )
            if len(command_packet) != 13:
                self._log.critical("Incomplete command packet")
            else:
                K = command_packet[11]

                if K in [0x0F, 0x1F]:
                    if K == 0x0F:
                        self._log.debug("Stop command received")
                    elif K == 0x1F:
                        self._log.debug("Status command received")

                    # convert to byte values
                    H = "00000" + str(round(float(self._az + 360), 1))
                    V = "00000" + str(round(float(self._el + 360), 1))

                    response_packet = [
                        0x57,
                        int(H[-5]),
                        int(H[-4]),
                        int(H[-3]),
                        int(H[-1]),
                        self._pulses_per_degree,
                        int(V[-5]),
                        int(V[-4]),
                        int(V[-3]),
                        int(V[-1]),
                        self._pulses_per_degree,
                        0x20,
                    ]

                    self._log.debug("Response queued")
                    self._log.debug("-> Azimuth:   " + str(self._az) + "°")
                    self._log.debug("-> Elevation: " + str(self._el) + "°")
                    self._log.debug(
                        "-> AZ_DIVISOR:        " + hex(self._pulses_per_degree)
                    )
                    self._log.debug(
                        "-> EL_DIVISOR:        " + hex(self._pulses_per_degree)
                    )

                    self._ser.write(bytearray(response_packet))

                    self._log.debug(
                        "Response packet sent: "
                        + str(list(map(hex, list(response_packet))))
                    )
                elif K == 0x2F:
                    # convert from ascii characters
                    H = (
                        ((command_packet[1] - 0x30) * 1000)
                        + ((command_packet[2] - 0x30) * 100)
                        + ((command_packet[3] - 0x30) * 10)
                        + (command_packet[4] - 0x30)
                    )
                    V = (
                        ((command_packet[6] - 0x30) * 1000)
                        + ((command_packet[7] - 0x30) * 100)
                        + ((command_packet[8] - 0x30) * 10)
                        + (command_packet[9] - 0x30)
                    )

                    # decode with resolution
                    self._az = H / self._pulses_per_degree - 360.0
                    self._el = V / self._pulses_per_degree - 360.0

                    self._az = float(round(self._az, 1))
                    self._el = float(round(self._el, 1))

                    self._log.debug("Set command received")
                    self._log.debug("-> Azimuth:   " + str(self._az) + "°")
                    self._log.debug("-> Elevation: " + str(self._el) + "°")
                else:
                    self._log.error(
                        "Invalid command received (K = " + str(hex(K)) + ")"
                    )

    def stop(self):
        """Stops the daemon thread running the simulator."""
        self._keep_running = False
