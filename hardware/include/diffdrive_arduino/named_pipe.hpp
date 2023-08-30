#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>

class NamedPipe
{
private:
    int fd = 0;
    char *fifo_path = (char *)"/tmp/ros2_control_out";
    char arr1[80], arr2[80];

public:
    NamedPipe(/* args */);
    ~NamedPipe();
    int init();
    int writeLine(std::string, bool);
};

NamedPipe::NamedPipe(/* args */)
{
    mkfifo(fifo_path, 0666);
}

NamedPipe::~NamedPipe()
{
    close(fd);
}

int NamedPipe::init()
{
    if (fd == 0)
    {
        // fd = open(fifo_path, O_WRONLY);
        fd = open(fifo_path, O_RDWR);
        std::cout << "pipe " << fd << " opened" << std::endl;
    }
    return 1;
}

int NamedPipe::writeLine(std::string str, bool printoutput = false)
{
    this->init();
    std::string write_str = str + '\n';
    char *c = strcpy(new char[write_str.length() + 1], write_str.c_str());

    // fd = open(fifo_path, O_RDONLY);
    // read(fd, arr1, strlen(arr1) + 1);
    // std::cout << "read pipe: " << arr1;
    // close(fd);
    // fd = open(fifo_path, O_WRONLY);
    int res = write(fd, c, strlen(c) + 1);
    // close(fd);
    if (printoutput)
        std::cout << "writed to pipe: " << str << "result: " << res << std::endl;

    return 1;
}
