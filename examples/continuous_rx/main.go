package main

import (
	"log"
	"time"

	"github.com/bemasher/sx1276"
)

func init() {
	log.SetFlags(log.Lshortfile | log.Lmicroseconds)
}

func main() {
	// Instantiate a new SX1276
	sx, err := sx1276.NewSX1276()
	if err != nil {
		log.Fatal(err)
	}
	defer sx.Close()

	// Receive for 10 seconds.
	after := time.After(10 * time.Second)

	// StartRxContinuous produces a channel of byte slices on which packets
	// will be sent.
	pkts := sx.StartRxContinuous()

	for {
		// Either receive a packet or timeout.
		select {
		case pkt := <-pkts:
			log.Printf("%q\n", pkt)
		case <-after:
			sx.StopRxContinuous()
			return
		}
	}
}
