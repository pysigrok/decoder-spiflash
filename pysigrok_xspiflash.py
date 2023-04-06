"""PySigrok decoder for multibit SPI flash transactions."""

__version__ = "0.0.1"

import sigrokdecode as srd

import struct

# value is dummy clocks
CONTINUE_COMMANDS = {
    0x6b: 8,
    0xe7: 2,
    0xeb: 4,
}

DATA_COMMANDS = {0x03: "Read",
                 0x0b: "Fast Read",
                 0x5b: "Read SFDP",
                 0x6b: "Quad-Output Fast Read",
                 0x9e: "Read JEDEC ID",
                 0x9f: "Read JEDEC ID",
                 0xe7: "Quad Word Read",
                 0xeb: "Quad Read",
                 0x02: "Page Program",
                 0x32: "Quad Page Program"}

EN4B = 0xB7
EX4B = 0xE9
CONTROL_COMMANDS = {
    0x01: "Write Status Register 1",
    0x06: "Write Enable",
    0x04: "Write Disable",
    0x05: "Read Status Register",
    0x35: "Read Status Register 2",
    0x5A: "Read SFDP Mode",
    0x75: "Program Suspend",
    0xAB: "Release Power-down / Device ID",
    EN4B: "Enable 4 Byte Address",
    EX4B: "Exit 4 Byte Address"
}

class Decoder(srd.Decoder):
    api_version = 3
    id = 'xspiflash'
    name = 'xSPI Flash'
    longname = 'SPI Flash including multibit busses'
    desc = 'Full-duplex, synchronous, serial bus.'
    license = 'MIT'
    inputs = ['logic']
    outputs = ['spiflash']
    tags = ['Embedded/industrial']
    channels = (
        {'id': 'cs', 'name': 'CS#', 'desc': 'Chip-select'},
        {'id': 'clk', 'name': 'CLK', 'desc': 'Clock'},
    )
    optional_channels = (
        {'id': 'miso', 'name': 'MISO', 'desc': 'MISO / D0'},
        {'id': 'mosi', 'name': 'MOSI', 'desc': 'MOSI / D1'},
        {'id': 'd2', 'name': 'D2', 'desc': 'MOSI / D1'},
        {'id': 'd3', 'name': 'D3', 'desc': 'MOSI / D1'},
    )
    options = (
        {'id': 'cs_polarity', 'desc': 'CS# polarity', 'default': 'active-low',
            'values': ('active-low', 'active-high')},
        {'id': 'cpol', 'desc': 'Clock polarity', 'default': 0,
            'values': (0, 1)},
        {'id': 'cpha', 'desc': 'Clock phase', 'default': 0,
            'values': (0, 1)},
        {'id': 'bitorder', 'desc': 'Bit order',
            'default': 'msb-first', 'values': ('msb-first', 'lsb-first')},
        {'id': 'wordsize', 'desc': 'Word size', 'default': 8},
        {'id': "address_bytes", 'desc': 'Number of bytes in address', 'default': 3, 'values': (1, 2, 3, 4)},
        {'id': 'min_address', 'desc': 'Minimum address to annotate', 'default': 0},
        {'id': 'max_address', 'desc': 'Maximum address to annotate', 'default': 0xffffffff}
    )
    annotations = (
        ('miso-data', 'MISO data'),
        ('mosi-data', 'MOSI data'),
        ('warning', 'Warning'),
        ('miso-transfer', 'MISO transfer'),
        ('mosi-transfer', 'MOSI transfer'),

        # Annotation types from the Logic 2 decoder
        ('control-command', "Control command"),
        ('data-command', "Data command"),
        ('xspi-error', "xSPI error")
    )
    annotation_rows = (
        ('miso-data-vals', 'MISO data', (0,)),
        ('miso-transfers', 'MISO transfers', (3,)),
        ('mosi-data-vals', 'MOSI data', (1,)),
        ('mosi-transfers', 'MOSI transfers', (4,)),
        ('other', 'Other', (2,)),
        ('commands', 'Commands', (5, 6)),
        ('error', 'Errors', (7,))
    )
    binary = (
        ('miso', 'MISO'),
        ('mosi', 'MOSI'),
    )

    def reset(self):
        self._start_time = None
        self._address_bytes = self.options["address_bytes"]
        self._address_format = "{:0" + str(2*int(self._address_bytes)) + "x}"
        self._min_address = int(self.options["min_address"])
        self._max_address = int(self.options["max_address"])

        self._miso_data = None
        self._mosi_data = None
        self._empty_result_count = 0

        # These are for quad decoding. The input will be a SimpleParallel analyzer
        # with the correct clock edge. CS is inferred from a gap in time.
        self._last_cs = 1
        self._last_time = None
        self._transaction = 0
        self._clock_count = 0
        self._mosi_out = 0
        self._miso_in = 0
        self._quad_data = 0
        self._quad_start = None
        self._continuous = False
        self._dummy = 0

    def start(self):
        self.out_python = self.register(srd.OUTPUT_PYTHON)
        self.out_ann = self.register(srd.OUTPUT_ANN)
        self.out_binary = self.register(srd.OUTPUT_BINARY)

    def put_bytes(self, ss, es, mosi=0, miso=0):
        self.put(ss, es, self.out_binary, [0, miso])
        self.put(ss, es, self.out_binary, [1, mosi])

        self.put(ss, es, self.out_python, ['DATA', miso, mosi])

        self.put(ss, es, self.out_ann, [0, ['%02X' % miso]])
        self.put(ss, es, self.out_ann, [1, ['%02X' % mosi]])

    def decode(self):
        clk_cond = [{0: "r"}, {1: "r"}]
        while True:
            self.wait([{0: "f"}])
            # start of transaction
            self._transaction += 1
            self._start_time = self.samplenum
            self._miso_data = bytearray()
            self._mosi_data = bytearray()
            self._quad_data = 0
            if not self._continuous:
                self._command = 0
                self._quad_start = None
                self._dummy = 0

                # Zero the data buffers to prevent issues with odd lengths of transactions if QSPI mode isn't detected properly.
                self._mosi_out = 0
                self._miso_in = 0
            else:
                self._clock_count = 8
                self._miso_data.append(0)
                self._mosi_data.append(self._command)

            cs, clk, miso, mosi, d2, d3 = self.wait(clk_cond)
            self._clock_count = 0
            self._word_start = None
            while self.matched[1]:
                if self._word_start is None:
                    self._word_start = self.samplenum
                # print(self.samplenum, cs, clk, miso, mosi, d2, d3)
                if self._quad_start is None or self._clock_count < self._quad_start:
                    self._mosi_out = self._mosi_out << 1 | mosi
                    self._miso_in = self._miso_in << 1 | miso
                    if self._clock_count % 8 == 7:
                        if self._clock_count == 7:
                            self._command = self._mosi_out
                            if self._command in CONTINUE_COMMANDS:
                                self._quad_start = 8
                                self._dummy = CONTINUE_COMMANDS[self._command]

                        self._mosi_data.append(self._mosi_out)
                        self._miso_data.append(self._miso_in)
                        # self.put_bytes(self._word_start, self.samplenum, self._mosi_out, self._miso_in)
                        self._mosi_out = 0
                        self._miso_in = 0
                        self._word_start = None
                else:
                    data = d3 << 3 | d2 << 2 | miso << 1 | mosi
                    self._quad_data = (self._quad_data << 4 | data & 0xf)
                    if self._clock_count % 2 == 1:
                        if not 15 < self._clock_count <= 15 + self._dummy:
                            self._mosi_data.append(self._quad_data)
                            self._miso_data.append(0)
                            # self.put_bytes(self._word_start, self.samplenum, mosi=self._quad_data)
                        else:
                            self._mosi_data.append(0)
                            self._miso_data.append(self._quad_data)
                            # self.put_bytes(self._word_start, self.samplenum, miso=self._quad_data)
                        if self._command in CONTINUE_COMMANDS and self._clock_count == 15:
                            # At least some SPI flashes use 'nibbles are complements' to enter
                            # continous read mode (or ST calls 'send instruction only'). So this
                            # should check for e.g., 0xa5. Unclear if some flashes don't do this
                            # and just use any pattern in high nibble, so check for 0xA in high
                            # nibble which seems to work in practice. If you aren't seeing
                            # continous reads working look here first.
                            self._continuous = (self._quad_data & 0xf0) == 0xa0
                        self._quad_data = 0
                        self._word_start = None

                cs, clk, miso, mosi, d2, d3 = self.wait(clk_cond)
                self._clock_count += 1

            if not self._miso_data or not self._mosi_data:
                continue
            command = self._mosi_data[0]

            if command in DATA_COMMANDS:
                if len(self._mosi_data) < 1 + int(self._address_bytes):
                    output_type = 7
                    message = "Error!"
                else:
                    output_type = 6
                    frame_address = 0
                    for i in range(int(self._address_bytes)):
                        frame_address <<= 8
                        frame_address += self._mosi_data[1+i]
                    if self._min_address <= frame_address <= self._max_address:
                        start_address = self._address_format.format(frame_address)
                        non_data_bytes = 1
                        # Fast read has a dummy byte
                        if DATA_COMMANDS[command] == DATA_COMMANDS[0x0b]:
                            non_data_bytes += 1
                        # QSPI read has dummy (4 cycles = 2bytes)
                        if DATA_COMMANDS[command] == DATA_COMMANDS[0xeb]:
                            non_data_bytes += 2
                        num_data_bytes = len(self._mosi_data) - int(self._address_bytes) - non_data_bytes
                        end_address = self._address_format.format(frame_address + num_data_bytes - 1)
                    message = f"{DATA_COMMANDS[command]} 0x{start_address} - 0x{end_address} ({num_data_bytes} data bytes)"
            else:
                output_type = 5
                if command in CONTROL_COMMANDS:
                    message = CONTROL_COMMANDS[command]
                else:
                    # Unrecognized commands are printed in hexadecimal
                    message = f"0x{command:X}"
                if command == EN4B:
                    self._address_bytes = 4
                    self._address_format = "{:0" + str(2*int(self._address_bytes)) + "x}"
                elif command == EX4B:
                    self._address_bytes = 3
                    self._address_format = "{:0" + str(2*int(self._address_bytes)) + "x}"

            if output_type != 5:
                self.put(self._start_time, self.samplenum, self.out_ann, [output_type, [message]])

