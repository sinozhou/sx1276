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
	last := time.Now()

	for {
		// Either receive a packet or timeout.
		select {
		case pkt := <-pkts:
			now := time.Now()
			log.Printf("%q, %s\n", pkt, now.Sub(last))
			last = now
		case <-after:
			sx.StopRxContinuous()
			return
		}
	}
}
