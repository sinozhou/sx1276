package sx1276

import (
	"errors"
	"log"
	"time"

	"github.com/kidoman/embd"
	_ "github.com/kidoman/embd/host/rpi"
)

const (
	RegFifo                = 0x00 // FIFO read/write access
	RegOpMode              = 0x01 // Operating mode and LoRa / FSK selection
	RegFrfMsb              = 0x06 // RF Carrier Frequency
	RegFrfMid              = 0x07 // RF Carrier Frequency
	RegFrfLsb              = 0x08 // RF Carrier Frequency, fRF = (F(XOSC)*Frf) / 1<<19
	RegPaConfig            = 0x09 // PA selection and Output Power control
	RegPaRamp              = 0x0A // Control of PA ramp time, low phase noise PLL
	RegOcp                 = 0x0B // Over Current Protection control
	RegLna                 = 0x0C // LNA settings
	RegFifoAddrPtr         = 0x0D // FIFO SPI pointer
	RegFifoTxBaseAddr      = 0x0E // Start Tx data
	RegFifoRxBaseAddr      = 0x0F // Start Rx data
	RegFifoRxCurrentAddr   = 0x10 // Start address of last packet received
	RegIrqFlagsMask        = 0x11 // Optional IRQ flag mask
	RegIrqFlags            = 0x12 // IRQ flags
	RegRxNbBytes           = 0x13 // Number of received bytes
	RegRxHeaderCntValueMsb = 0x14 // Number of valid headers received
	RegRxHeaderCntValueLsb = 0x15 // Number of valid headers received
	RegRxPacketCntValueMsb = 0x16 // Number of valid packets received
	RegRxPacketCntValueLsb = 0x17 // Number of valid packets received
	RegModemStat           = 0x18 // Live LoRa modem status
	RegPktSnrValue         = 0x19 // Estimation of last packet SNR
	RegPktRssiValue        = 0x1A // RSSI of last packet
	RegRssiValue           = 0x1B // Current RSSI
	RegHopChannel          = 0x1C // FHSS start channel
	RegModemConfig1        = 0x1D // Modem PHY config
	RegModemConfig2        = 0x1E // Modem PHY config
	RegSymbTimeoutLsb      = 0x1F // Recevier timeout value
	RegPreambleMsb         = 0x20 // Size of Preamble
	RegPreambleLsb         = 0x20 // Size of Preamble
	RegPayloadLength       = 0x22 // LoRa payload length
	RegMaxPayloadLength    = 0x23 // LoRa maximum payload length
	RegHopPeriod           = 0x24 // FHSS Hop period
	RegFifoRxByteAddr      = 0x25 // Address of last byte written in FIFO
	RegModemConfig3        = 0x26 // Modem PHY config
	RegFeiMsb              = 0x28 // Estimated frequency error
	RegFeiMid              = 0x29 // Estimated frequency error
	RegFeiLsb              = 0x2A // Estimated frequency error
	RegRssiWideband        = 0x2C // Wideband RSSI measurement
	RegDetectOptimize      = 0x31 // LoRa detection Optimize for SF6
	RegInvertIQ            = 0x33 // Invert LoRa I and Q
	RegDetectionThreshold  = 0x37 // LoRa detection theshold for SF6
	RegSyncWord            = 0x39 // LoRa Sync Word
	RegDioMapping1         = 0x40 // Mapping of pins DIO0 to DIO3
	RegDioMapping2         = 0x41 // Mapping of pins DIO4 and DIO5, ClkOut frequency
	RegVersion             = 0x42 // Semtech ID relating the silicon revision
	RegTcxo                = 0x4B // TCXO or XTAL input setting
	RegPaDac               = 0x4D // Higher power settings of the PA
	RegFormerTemp          = 0x5B // Stored temperature during the former IQ Calibration
	RegAgcRef              = 0x61 // Adjustment of the AGC thresholds
	RegAgcThresh1          = 0x62 // Adjustment of the AGC thresholds
	RegAgcThresh2          = 0x63 // Adjustment of the AGC thresholds
	RegAgcThresh3          = 0x64 // Adjustment of the AGC thresholds
	RegPll                 = 0x70 // Control of the PLL bandwidth
)

const (
	FifoTxBaseAddr = 0x02
	FifoRxBaseAddr = 0x0A
)

const (
	SS   = 6
	DIO0 = 7
	RST  = 0
)

type SX1276 struct {
	spi embd.SPIBus

	ss   embd.DigitalPin
	dio0 embd.DigitalPin
	rst  embd.DigitalPin

	IrqDIO0          chan struct{}
	stopRxContinuous chan struct{}
}

// Returns a new SX1276 controller. If an error is encountered, the returned controller
// will be nil.
func NewSX1276() (sx *SX1276, err error) {
	// Currently only Raspberry Pi 3's are supported.
	embd.SetHost(embd.HostRPi, 3)

	sx = new(SX1276)

	// Initialize SPI
	if err := embd.InitSPI(); err != nil {
		return nil, err
	}

	sx.spi = embd.NewSPIBus(embd.SPIMode0, 0, 115200, 8, 0)

	// Initialize GPIO
	if err := embd.InitGPIO(); err != nil {
		return nil, err
	}

	// Setup SPI chip-select pin.
	sx.ss, _ = embd.NewDigitalPin("GPIO_25")
	sx.ss.SetDirection(embd.Out)
	sx.ss.Write(embd.High)

	// Setup DIO0 interrupt.
	sx.dio0, _ = embd.NewDigitalPin("GPIO_4")
	sx.dio0.SetDirection(embd.In)
	sx.IrqDIO0 = make(chan struct{})
	err = sx.dio0.Watch(embd.EdgeRising, func(pin embd.DigitalPin) {
		sx.IrqDIO0 <- struct{}{}
	})

	// Setup RST pin.
	sx.rst, _ = embd.NewDigitalPin("GPIO_17")
	err = sx.rst.SetDirection(embd.Out)

	// Reset device.
	sx.rst.Write(embd.Low)
	time.Sleep(10 * time.Microsecond)
	sx.rst.Write(embd.High)
	time.Sleep(5 * time.Millisecond)

	// Check chip version.
	// SX1276's should return 0x12
	if val, err := sx.ReadReg(RegVersion); err != nil {
		return nil, err
	} else {
		if val != 0x12 {
			return nil, errors.New("invalid receiver version")
		}
	}

	// Enable LoRa and set the transceiver's mode to standby.
	sx.WriteReg(RegOpMode, 0x80|byte(SLEEP))

	// Set default frequency.
	err = sx.SetFrequency(904500000)
	if err != nil {
		return nil, err
	}

	// Set maximum payload length and configure output power.
	err = sx.WriteReg(RegMaxPayloadLength, 0x80)
	if err != nil {
		return nil, err
	}

	err = sx.WriteReg(RegPaConfig, 0xCF)

	return
}

func (sx SX1276) Close() {
	// Clean up SPI connections.
	sx.spi.Close()
	embd.CloseSPI()

	// Clean up GPIO pins.
	sx.ss.Close()
	sx.dio0.Close()
	sx.rst.Close()
	embd.CloseGPIO()
}

// Reads the register at addr.
func (sx SX1276) ReadReg(addr byte) (byte, error) {
	sx.ss.Write(embd.Low)
	defer sx.ss.Write(embd.High)

	buf := []byte{addr & 0x7F, 0}
	err := sx.spi.TransferAndReceiveData(buf)

	return buf[1], err
}

// Writes val to the register at addr.
func (sx SX1276) WriteReg(addr, val byte) error {
	sx.ss.Write(embd.Low)
	defer sx.ss.Write(embd.High)

	buf := []byte{addr | 0x80, val}

	return sx.spi.TransferAndReceiveData(buf)
}

// Reads n number of values from the registers beginning at addr, increasing sequentially.
func (sx SX1276) ReadRegBurst(addr byte, n int) ([]byte, error) {
	if n < 0 || n > 0xFF || int(addr)+n > 0xFF {
		return nil, errors.New("invalid number of registers")
	}

	sx.ss.Write(embd.Low)
	defer sx.ss.Write(embd.High)

	buf := make([]byte, n+1)
	buf[0] = addr & 0x7F
	err := sx.spi.TransferAndReceiveData(buf)

	return buf[1:], err
}

// Writes val to the registers beginning at addr, increasing sequentially.
func (sx SX1276) WriteRegBurst(addr byte, val ...byte) error {
	if len(val) > 255 {
		return errors.New("invalid number of registers to write")
	}

	sx.ss.Write(embd.Low)
	defer sx.ss.Write(embd.High)

	buf := append([]byte{addr | 0x80}, val...)

	return sx.spi.TransferAndReceiveData(buf)
}

type OpMode byte

const (
	SLEEP OpMode = iota
	STDBY
	FSTX
	TX
	FSRX
	RXCONTINUOUS
	RXSINGLE
	CAD
)

// Set transceiver's operating mode.
func (sx SX1276) SetOpMode(mode OpMode) error {
	if mode > CAD {
		return errors.New("invalid operating mode")
	}

	val, err := sx.ReadReg(RegOpMode)
	if err != nil {
		return err
	}

	return sx.WriteReg(RegOpMode, (val&0xF8)|byte(mode))
}

func (sx SX1276) SetFrequency(freq float64) error {
	if freq < 137e6 || freq > 1020e6 {
		return errors.New("invalid frequency")
	}

	uFreq := (uint64(freq) << 19) / 32000000

	return sx.WriteRegBurst(RegFrfMsb, byte(uFreq>>16), byte(uFreq>>8), byte(uFreq))
}

func (sx SX1276) GetFrequency() (float64, error) {
	freq, err := sx.ReadRegBurst(RegFrfMsb, 3)
	if err != nil {
		return 0, err
	}

	return float64(((uint64(freq[0])<<16 + uint64(freq[1])<<8 + uint64(freq[2])) * 32000000) >> 19), err
}

type Bandwidth byte

const (
	BW7_8   Bandwidth = iota //   7.8 kHz
	BW10_4                   //  10.4
	BW15_6                   //  15.6
	BW20_8                   //  20.8
	BW31_25                  //  31.25
	BW41_7                   //  41.7
	BW62_5                   //  62.5
	BW125                    // 125
	BW250                    // 250
	BW500                    // 500
)

func (sx SX1276) SetBandwidth(bw Bandwidth) error {
	if bw > BW500 {
		return errors.New("invalid bandwidth")
	}
	regModemConfig1, err := sx.ReadReg(RegModemConfig1)
	if err != nil {
		return err
	}

	return sx.WriteReg(RegModemConfig1, (regModemConfig1&0x0F)|byte(bw)<<4)
}

type CodingRate byte

const (
	CR45 CodingRate = iota + 1 // 4/5 = 1.25 overhead ratio
	CR46                       // 4/6 = 1.50
	CR47                       // 4/7 = 1.75
	CR48                       // 4/8 = 2.00
)

func (sx SX1276) SetCodingRate(cr CodingRate) error {
	if cr < CR45 || cr > CR48 {
		return errors.New("invalid coding rate")
	}
	regModemConfig1, err := sx.ReadReg(RegModemConfig1)
	if err != nil {
		return err
	}

	return sx.WriteReg(RegModemConfig1, (regModemConfig1&0xF1)|byte(cr)<<1)
}

func (sx SX1276) SetImplicitHeader(enable bool) error {
	regModemConfig1, err := sx.ReadReg(RegModemConfig1)
	if err != nil {
		return err
	}

	if enable {
		return sx.WriteReg(RegModemConfig1, regModemConfig1|0x01)
	}
	return sx.WriteReg(RegModemConfig1, regModemConfig1&0xFE)
}

type SpreadingFactor byte

const (
	SF6  SpreadingFactor = iota + 6 // 64 chips / symbol
	SF7                             // 128
	SF8                             // 256
	SF9                             // 512
	SF10                            // 1024
	SF11                            // 2048
	SF12                            // 4096
)

func (sx SX1276) SetSpreadingFactor(sf SpreadingFactor) error {
	if sf < SF6 || sf > SF12 {
		return errors.New("invalid spreading factor")
	}

	regModemConfig2, err := sx.ReadReg(RegModemConfig2)
	if err != nil {
		return err
	}

	return sx.WriteReg(RegModemConfig2, (regModemConfig2&0x0F)|byte(sf)<<4)
}

func (sx SX1276) LastPktRSSI() (float64, error) {
	// We're assuming HF port here.
	rssi, err := sx.ReadReg(RegPktRssiValue)
	return -157 + float64(rssi), err
}

func (sx SX1276) LastPktSNR() (float64, error) {
	snr, err := sx.ReadReg(RegPktSnrValue)
	return (float64(snr) - 127) / 4.0, err
}

// Transmit payload. This method assumes the transceiver's operating mode is
// either SLEEP or STDBY.
func (sx SX1276) Tx(payload []byte) (err error) {
	// Enable TxDone interrupt on DIO0.
	err = sx.WriteReg(RegDioMapping1, 0x40)
	if err != nil {
		return
	}

	// Set the FIFO's starting address.
	err = sx.WriteReg(RegFifoAddrPtr, FifoTxBaseAddr)
	if err != nil {
		return
	}

	// Set the number of bytes to be transmitted.
	err = sx.WriteReg(RegPayloadLength, byte(len(payload)))
	if err != nil {
		return
	}

	// Write the payload to the FIFO.
	err = sx.WriteRegBurst(RegFifo, payload...)
	if err != nil {
		return
	}

	// Begin transmission.
	err = sx.SetOpMode(TX)
	if err != nil {
		return
	}

	// Wait for TxDone
	<-sx.IrqDIO0

	// Clear TxDone interrupt on DIO0.
	err = sx.WriteReg(RegIrqFlags, 0x08)
	if err != nil {
		return
	}

	return nil
}

func (sx SX1276) rx() ([]byte, error) {
	// Get the current IRQ flags.
	irqFlags, err := sx.ReadReg(RegIrqFlags)
	if err != nil {
		return nil, err
	}

	// If the PayloadCrcError interrupt flag is asserted, clear the flag and
	// return an error.
	if irqFlags&0x20 == 0x20 {
		err := sx.WriteReg(RegIrqFlags, 0x20)
		if err != nil {
			return nil, err
		}

		return nil, errors.New("bad crc")
	}

	// Get the current Rx FIFO starting address.
	currentAddr, err := sx.ReadReg(RegFifoRxCurrentAddr)
	if err != nil {
		return nil, err
	}

	// Get the number of received bytes.
	receivedBytes, err := sx.ReadReg(RegRxNbBytes)
	if err != nil {
		return nil, err
	}

	// Move the FIFO to the beginning of the received bytes.
	err = sx.WriteReg(RegFifoAddrPtr, currentAddr)
	if err != nil {
		return nil, err
	}

	// Read the payload from the FIFO.
	payload, err := sx.ReadRegBurst(RegFifo, int(receivedBytes))

	return payload, err
}

// Start receiving packets continuously. Sends received packets on the
// returned channel. This method will panic if another continuous receive
// operation is already running.
func (sx *SX1276) StartRxContinuous() (pkts chan []byte) {
	pkts = make(chan []byte)

	// If sx.stopRxContinuous isn't nil, we're already receiving, panic.
	if sx.stopRxContinuous != nil {
		panic("already continuously receiving")
	}
	sx.stopRxContinuous = make(chan struct{})

	// Enable RxDone interrupt on DIO0
	sx.WriteReg(RegDioMapping1, 0x00)

	// Start receiving.
	sx.SetOpMode(RXCONTINUOUS)

	go func() {
		for {
			select {
			case <-sx.IrqDIO0:
				// DIO0's interrupt has occured. Clear RxDone IRQ.
				sx.WriteReg(RegIrqFlags, 0x40)
				// Receive the packet's payload or print an error.
				if pkt, err := sx.rx(); err != nil {
					log.Println(err)
				} else {
					pkts <- pkt
				}
			case <-sx.stopRxContinuous:
				// Reception has been requested to stop.
				sx.stopRxContinuous <- struct{}{}
				return
			}
		}
	}()

	return
}

// Stop continuous reception.
func (sx SX1276) StopRxContinuous() {
	// Set mode to standby.
	sx.SetOpMode(STDBY)
	// Signal that the receiving goroutine should exit.
	sx.stopRxContinuous <- struct{}{}
	// Wait for goroutine to exit.
	<-sx.stopRxContinuous
	// Set the stop channel to nil for future use.
	sx.stopRxContinuous = nil
}
