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
    int writePipe(char *, bool);
    int writeLine(std::string, bool);
};

NamedPipe::NamedPipe(/* args */)
{
    mkfifo(this->fifo_path, 0666);
}

NamedPipe::~NamedPipe()
{
    close(this->fd);
}

int NamedPipe::init()
{
    this->fd = open(this->fifo_path, O_WRONLY);
    return 1;
}

int NamedPipe::writePipe(char *arr2, bool printoutput = false)
{
    if (this->fd == 0)
        this->init();
    write(fd, arr2, strlen(arr2) + 1);
    if (printoutput)
        std::cout << "writed to pipe: " << arr2 << std::endl;
    return 1;
}

int NamedPipe::writeLine(std::string str, bool printoutput = false)
{
    if (this->fd == 0)
        this->init();
    std::string write_str = str + '\n';
    char *c = strcpy(new char[write_str.length() + 1], write_str.c_str());
    write(fd, c, strlen(c) + 1);
    if (printoutput)
        std::cout << "writed to pipe: " << str << std::endl;
    return 1;
}
