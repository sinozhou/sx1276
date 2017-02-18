package main

import (
	"flag"
	"log"
	"time"

	"github.com/bemasher/sx1276"
)

func init() {
	log.SetFlags(log.Lshortfile | log.Lmicroseconds)
}

func main() {
	duration := flag.Duration("duration", 10*time.Second, "length of time to listen, 0 to listen forever")

	flag.Parse()

	// Instantiate a new SX1276
	sx, err := sx1276.NewSX1276()
	if err != nil {
		log.Fatal(err)
	}
	defer sx.Close()

	// Receive for 10 seconds.
	after := make(<-chan time.Time)
	if *duration > time.Duration(0) {
		after = time.After(*duration)
	}

	// StartRxContinuous produces a channel of byte slices on which packets
	// will be sent.
	pkts := sx.StartRxContinuous()

	for {
		// Either receive a packet or timeout.
		select {
		case pkt := <-pkts:
			rssi, _ := sx.LastPktRSSI()
			snr, _ := sx.LastPktSNR()

			log.Printf("%q, RSSI: %0.0f SNR: %0.1f\n", pkt, rssi, snr)
		case <-after:
			sx.StopRxContinuous()
			return
		}
	}
}
