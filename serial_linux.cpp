#include <iostream>      // standard input / output functions
#include <stdlib.h>
#include <string>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <cstring>
#include <mutex>
#include <pthread.h>

using std::cout;
using std::cin;
using std::endl;
using std::getline;
using std::mutex;
using std::string;

mutex mtx;
int port;

int openPort(string portName) 
{
    /* Open File Descriptor */
    int port = open(portName.c_str(), O_RDWR | O_NOCTTY);

    /* Error Handling */
    if (port < 0)
    {
        cout << "Error " << errno << " opening " << portName << ": " << strerror(errno) << endl;
    }

    /* *** Configure Port *** */
    struct termios tty;
    memset(&tty, 0, sizeof tty);

    /* Error Handling */
    if (tcgetattr(port, &tty) != 0)
    {
        cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
    }

    /* Set Baud Rate */
    cfsetospeed(&tty, B19200);
    cfsetispeed(&tty, B19200);

    /* Setting other Port Stuff */
    tty.c_cflag &= ~PARENB;        // Make 8n1
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;       // no flow control
    tty.c_lflag = 0;          // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                  // no remapping, no delays
    tty.c_cc[VMIN] = 0;                  // read doesn't block
    tty.c_cc[VTIME] = 5;                  // 0.5 seconds read timeout

    tty.c_cflag |= CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);// turn off s/w flow ctrl
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    tty.c_oflag &= ~OPOST;              // make raw

    /* Flush Port, then applies attributes */
    tcflush(port, TCIFLUSH);

    if (tcsetattr(port, TCSANOW, &tty) != 0)
    {
        cout << "Error " << errno << " from tcsetattr" << endl;
    }

    return port;
}

int writePort(int port, const char* cmd)
{
    int n_written = 0, spot = 0;

    do {
        n_written = write(port, &cmd[spot], 1);
        spot += n_written;
    } while (cmd[spot - 1] != '\r' && n_written > 0);

    return 1;
}

int readPort(int port)
{
    int n = 1;
    char buf = '\0';

    /* Whole response*/
    string text;

    do {
        n = read(port, &buf, 1);
        text.push_back(buf);
    } while (buf != '\r' && n > 0);

    if (n < 0) {
        mtx.lock();
        cout << "Error reading: " << strerror(errno) << endl;
        mtx.unlock();
    }
    else if (buf != '\0') {
        mtx.lock();
        cout << "Received: " << text << endl;
        mtx.unlock();
    }

    return n;
}

void *lookForInput(void *ptr)
{
    while (readPort(port) >= 0);
}

int main()
{
    string portName;
    cout << "Input the port: ";
    getline(cin, portName);

    port = openPort(portName);

    if (port)
    {
        pthread_t reading;
        pthread_create(&reading, NULL, lookForInput, NULL);

        while (true) {
            string str;
            getline(cin, str);
            str.push_back('\r');
            writePort(port, str.c_str());
        }

        pthread_join(reading, NULL);

        int key;
        cout << "Press any key to continue..." << endl;
        cin >> key;
    }
    else
        cout << "Failed to open port!" << endl;

    return 0;
}