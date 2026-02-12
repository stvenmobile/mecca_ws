#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>
#include <iomanip>

int main() {
    const char* port = "/dev/stm32_serial";
    std::cout << "--- Pi 5 RP1 Deep Debug (v2.7.0) ---" << std::endl;

    // 1. Open with O_NONBLOCK initially to bypass any hardware handshake hangs
    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        perror("Open failed");
        return 1;
    }

    // 2. Clear NONBLOCK so we can use VTIME timeouts
    fcntl(fd, F_SETFL, 0);

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr failed");
        return 1;
    }

    // 3. Match PySerial's 'Raw' configuration exactly
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);    // Ignore modem, enable receiver
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;                 // 8-bit characters
    tty.c_cflag &= ~PARENB;             // No parity
    tty.c_cflag &= ~CSTOPB;             // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;            // No hardware flow control

    tty.c_lflag = 0;                    // No echo, no canonical, no signals
    tty.c_iflag = IGNPAR;               // Ignore parity errors
    tty.c_oflag = 0;                    // No output processing

    // 4. Set a generous timeout (1.0 seconds)
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10; 

    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr failed");
        return 1;
    }

    // 5. Force the RTS/DTR pins High (The 'Python' Magic)
    int status;
    ioctl(fd, TIOCMGET, &status);
    status |= TIOCM_DTR;
    status |= TIOCM_RTS;
    ioctl(fd, TIOCMSET, &status);

    while (true) {
        // Clear anything in the buffer before asking
        tcflush(fd, TCIFLUSH);

        std::cout << "Writing 'I ENC\\r\\n'..." << std::endl;
        const char* cmd = "I ENC\r\n";
        write(fd, cmd, strlen(cmd));
        
        // Wait for the TX buffer to be physically empty
        tcdrain(fd);

        char buf[256];
        memset(buf, 0, sizeof(buf));
        
        // Read call
        int n = read(fd, buf, sizeof(buf) - 1);
        
        if (n > 0) {
            std::cout << "SUCCESS! Received " << n << " bytes." << std::endl;
            std::cout << "Data: " << buf << std::endl;
        } else if (n == 0) {
            std::cout << "TIMEOUT: Port is silent." << std::endl;
        } else {
            std::cout << "ERROR: " << strerror(errno) << std::endl;
        }
        
        sleep(1);
    }

    close(fd);
    return 0;
}