package main

import (
	"flag"
	"log"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/bemasher/sx1276"
)

func init() {
	log.SetFlags(log.Lshortfile | log.Lmicroseconds)
}

func main() {
	sig := make(chan os.Signal)
	signal.Notify(sig, syscall.SIGINT, syscall.SIGTERM)

	duration := flag.Duration("duration", 10*time.Second, "length of time to listen, 0 to listen forever")
	flag.Parse()

	// Instantiate a new SX1276
	sx, err := sx1276.NewSX1276()
	if err != nil {
		log.Fatal(err)
	}
	defer sx.Close()

	// Receive for specified amount of time.
	after := make(<-chan time.Time)
	if *duration > time.Duration(0) {
		after = time.After(*duration)
	}

	// StartRxContinuous produces a channel of byte slices on which packets
	// will be sent.
	pkts := sx.StartRxContinuous()
	defer sx.StopRxContinuous()

	for {
		// Receive a packet, timeout or die.
		select {
		case pkt := <-pkts:
			rssi := sx.LastPktRSSI()
			snr := sx.LastPktSNR()
			dBm := sx.LastPktPower()

			log.Printf("%q, RSSI: %0.0f SNR: %0.1f dBm: %0.1f\n", pkt, rssi, snr, dBm)
		case <-after:
			return
		case <-sig:
			return
		}
	}
}
