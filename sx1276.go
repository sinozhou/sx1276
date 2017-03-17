package sx1276

import (
	"errors"
	"fmt"
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
	RegPreambleLsb         = 0x21 // Size of Preamble
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
	RST = "GPIO_22"
)

type SX1276 struct {
	spi embd.SPIBus

	rst embd.DigitalPin

	DIO0 InterruptPin
	DIO1 InterruptPin
	DIO2 InterruptPin
	DIO3 InterruptPin
	DIO4 InterruptPin
	DIO5 InterruptPin

	stopRxContinuous chan struct{}
}

type InterruptPin struct {
	embd.DigitalPin

	pinName string
	rpiName string

	Irq chan struct{}
}

func NewIntPin(pinName, rpiName string) (intPin InterruptPin, err error) {
	intPin.pinName = pinName
	intPin.Irq = make(chan struct{})

	intPin.DigitalPin, err = embd.NewDigitalPin(rpiName)
	if err != nil {
		return
	}

	intPin.SetDirection(embd.In)
	intPin.Watch(embd.EdgeRising, func(pin embd.DigitalPin) {
		select {
		case intPin.Irq <- struct{}{}:
		default:
		}
	})

	return
}

func (intPin InterruptPin) String() string {
	return fmt.Sprintf("{PinName: %s RPiName: %s}", intPin.pinName, intPin.rpiName)
}

// Returns a new SX1276 controller. If an error is encountered, the returned controller
// will be nil.
func NewSX1276() (sx *SX1276, err error) {
	// Currently only Raspberry Pi 3's are supported.
	embd.SetHost(embd.HostRPi, 0xA32082)

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

	// Setup RST pin.
	sx.rst, err = embd.NewDigitalPin(RST)
	if err != nil {
		return nil, err
	}
	err = sx.rst.SetDirection(embd.Out)
	if err != nil {
		return nil, err
	}

	// Reset the device.
	sx.rst.Write(embd.High)
	time.Sleep(10 * time.Millisecond)
	sx.rst.Write(embd.Low)
	time.Sleep(1 * time.Millisecond)
	sx.rst.Write(embd.High)
	time.Sleep(6 * time.Millisecond)

	// Check chip version. SX1276's should return 0x12
	if val := sx.ReadReg(RegVersion); val != 0x12 {
		return nil, errors.New("invalid receiver version")
	}

	// Put transceiver to sleep and enable LoRa.
	sx.WriteReg(RegOpMode, 0x88)

	// Setup DIO interrrupts.
	if sx.DIO0, err = NewIntPin("DIO0", "GPIO_12"); err != nil {
		return nil, err
	}
	if sx.DIO1, err = NewIntPin("DIO1", "GPIO_6"); err != nil {
		return nil, err
	}
	if sx.DIO2, err = NewIntPin("DIO2", "GPIO_5"); err != nil {
		return nil, err
	}
	if sx.DIO3, err = NewIntPin("DIO3", "GPIO_16"); err != nil {
		return nil, err
	}
	if sx.DIO4, err = NewIntPin("DIO4", "GPIO_13"); err != nil {
		return nil, err
	}
	if sx.DIO5, err = NewIntPin("DIO5", "GPIO_27"); err != nil {
		return nil, err
	}

	// Set default frequency.
	err = sx.SetFrequency(904500000)
	if err != nil {
		return nil, err
	}

	// Set maximum payload length and configure output power.
	sx.WriteReg(RegMaxPayloadLength, 0x80)
	sx.WriteReg(RegPaConfig, 0xCF)

	return
}

func (sx SX1276) Close() {
	// Clean up GPIO pins.
	sx.DIO0.Close()
	sx.DIO1.Close()
	sx.DIO2.Close()
	sx.DIO3.Close()
	sx.DIO4.Close()
	sx.DIO5.Close()

	sx.rst.Close()
	embd.CloseGPIO()

	// Clean up SPI connections.
	sx.spi.Close()
	embd.CloseSPI()
}

// Reads the register at addr.
func (sx SX1276) ReadReg(addr byte) byte {
	buf := []byte{addr & 0x7F, 0}
	err := sx.spi.TransferAndReceiveData(buf)
	if err != nil {
		panic(err)
	}

	return buf[1]
}

// Writes val to the register at addr.
func (sx SX1276) WriteReg(addr, val byte) {
	buf := []byte{addr | 0x80, val}
	err := sx.spi.TransferAndReceiveData(buf)
	if err != nil {
		panic(err)
	}
}

// Reads n number of values from the registers beginning at addr, increasing sequentially.
func (sx SX1276) ReadRegBurst(addr byte, n int) []byte {
	if n < 0 || n > 0xFF || int(addr)+n > 0xFF {
		panic(errors.New("invalid number of registers"))
	}

	buf := make([]byte, n+1)
	buf[0] = addr & 0x7F
	err := sx.spi.TransferAndReceiveData(buf)
	if err != nil {
		panic(err)
	}

	return buf[1:]
}

// Writes val to the registers beginning at addr, increasing sequentially.
func (sx SX1276) WriteRegBurst(addr byte, val ...byte) {
	if len(val) > 255 {
		panic(errors.New("invalid number of registers to write"))
	}

	buf := append([]byte{addr | 0x80}, val...)
	err := sx.spi.TransferAndReceiveData(buf)
	if err != nil {
		panic(err)
	}

	return
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

	// The ModeReady interrupt may occur on DIO5 before we've returned from
	// writing the new OpMode, so start listening _before_ we write anything.
	dio5 := make(chan struct{})
	go func() {
		dio5 <- <-sx.DIO5.Irq
	}()

	val := sx.ReadReg(RegOpMode)
	sx.WriteReg(RegOpMode, (val&0xF8)|byte(mode))
	<-dio5

	return nil
}

func (sx SX1276) SetFrequency(freq float64) error {
	if freq < 137e6 || freq > 1020e6 {
		return errors.New("invalid frequency")
	}

	uFreq := (uint64(freq) << 19) / 32000000
	sx.WriteRegBurst(RegFrfMsb, byte(uFreq>>16), byte(uFreq>>8), byte(uFreq))

	return nil
}

func (sx SX1276) GetFrequency() float64 {
	freq := sx.ReadRegBurst(RegFrfMsb, 3)

	return float64(((uint64(freq[0])<<16 + uint64(freq[1])<<8 + uint64(freq[2])) * 32000000) >> 19)
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
	regModemConfig1 := sx.ReadReg(RegModemConfig1)
	sx.WriteReg(RegModemConfig1, (regModemConfig1&0x0F)|byte(bw)<<4)

	return nil
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
	regModemConfig1 := sx.ReadReg(RegModemConfig1)
	sx.WriteReg(RegModemConfig1, (regModemConfig1&0xF1)|byte(cr)<<1)

	return nil
}

func (sx SX1276) SetImplicitHeader(enable bool) {
	regModemConfig1 := sx.ReadReg(RegModemConfig1)

	if enable {
		sx.WriteReg(RegModemConfig1, regModemConfig1|0x01)
	} else {
		sx.WriteReg(RegModemConfig1, regModemConfig1&0xFE)
	}
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

	regModemConfig2 := sx.ReadReg(RegModemConfig2)
	sx.WriteReg(RegModemConfig2, (regModemConfig2&0x0F)|byte(sf)<<4)

	return nil
}

// Enable Spreading Factor 6 specific register values.
func (sx SX1276) EnableSF6() {
	sx.SetSpreadingFactor(SF6) // Set Spreading Factor to SF6
	sx.SetImplicitHeader(true) // Enable ImplicitHeader

	// Set DetectionOptimize in RegDetectOptimize to the SF6 specific value.
	regDetectOptimize := sx.ReadReg(RegDetectOptimize)
	sx.WriteReg(RegDetectOptimize, (regDetectOptimize&0xF8)|0x05)

	// Set RegDetectionThreshold to the SF6 specific value.
	sx.WriteReg(RegDetectionThreshold, 0x0C)
}

// Reset Spreading Factor 6 specific register values for SF7-12 use.
func (sx SX1276) DisableSF6() {
	sx.SetSpreadingFactor(SF7)  // Set Spreading Factor to SF7
	sx.SetImplicitHeader(false) // Disable ImplicitHeader

	// Set DetectionOptimize in RegDetectOptimize to the SF7-12 specific value.
	regDetectOptimize := sx.ReadReg(RegDetectOptimize)
	sx.WriteReg(RegDetectOptimize, (regDetectOptimize&0xF8)|0x03)

	// Set RegDetectionThreshold to the SF7-12 specific value.
	sx.WriteReg(RegDetectionThreshold, 0x0A)
}

func (sx SX1276) LastPktRSSI() float64 {
	// We're assuming HF port here.
	rssi := sx.ReadReg(RegPktRssiValue)
	return -157 + float64(rssi)
}

func (sx SX1276) LastPktSNR() float64 {
	snr := sx.ReadReg(RegPktSnrValue)
	return (float64(snr) - 127) / 4.0
}

func (sx SX1276) LastPktPower() float64 {
	return sx.LastPktRSSI() + sx.LastPktSNR()
}

// Transmit payload. This method assumes the transceiver's operating mode is
// either SLEEP or STDBY.
func (sx SX1276) Tx(payload []byte) {
	sx.WriteReg(RegDioMapping1, 0x40)                 // Enable TxDone interrupt on DIO0.
	sx.WriteReg(RegFifoAddrPtr, FifoTxBaseAddr)       // Set the FIFO's starting address.
	sx.WriteReg(RegPayloadLength, byte(len(payload))) // Set the number of bytes to be transmitted.
	sx.WriteRegBurst(RegFifo, payload...)             // Write the payload to the FIFO.
	sx.SetOpMode(TX)                                  // Begin transmission.
	<-sx.DIO0.Irq                                     // Wait for TxDone on DIO0.
	sx.WriteReg(RegIrqFlags, 0x08)                    // Clear TxDone interrupt on DIO0.
}

func (sx SX1276) rx() ([]byte, error) {
	// Get the current IRQ flags.
	irqFlags := sx.ReadReg(RegIrqFlags)

	// If the PayloadCrcError interrupt flag is asserted, clear the flag and
	// return an error.
	if irqFlags&0x20 == 0x20 {
		sx.WriteReg(RegIrqFlags, 0x20)
		return nil, errors.New("bad crc")
	}

	rxAddr := sx.ReadReg(RegFifoRxCurrentAddr)        // Get the current Rx FIFO starting address.
	sx.WriteReg(RegFifoAddrPtr, rxAddr)               // Move the FIFO to the beginning of the received bytes.
	rxBytes := sx.ReadReg(RegRxNbBytes)               // Get the number of received bytes.
	payload := sx.ReadRegBurst(RegFifo, int(rxBytes)) // Read the payload from the FIFO.

	return payload, nil
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
			case <-sx.DIO0.Irq:
				// DIO0's interrupt has occured. Clear RxDone IRQ.
				sx.WriteReg(RegIrqFlags, 0x40)
				// Receive the packet's payload or print an error.
				if pkt, err := sx.rx(); err != nil {
					log.Println(err)
				} else {
					pkts <- pkt
				}
			case <-sx.stopRxContinuous:
				return
			}
		}
	}()

	return
}

// Stop continuous reception.
func (sx SX1276) StopRxContinuous() {
	sx.stopRxContinuous <- struct{}{} // Signal that the receiving goroutine should exit.
	sx.SetOpMode(STDBY)               // Set mode to standby.
	sx.stopRxContinuous = nil         // Set the stop channel to nil for future use.
}
