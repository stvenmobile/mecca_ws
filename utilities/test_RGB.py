import spidev
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 6400000
# Send a burst of data to see if any LED reacts
spi.xfer2([0xFF] * 100)
spi.close()