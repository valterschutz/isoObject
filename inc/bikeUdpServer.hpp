#include <iostream>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

class BikeUDPServer {
public:
    BikeUDPServer(boost::asio::io_context& io_context, int port)
        : socket_(io_context, udp::endpoint(udp::v4(), port))
    {
        start_receive();
    }

private:
    void start_receive()
    {
        socket_.async_receive_from(
            boost::asio::buffer(data_, max_length), remote_endpoint_,
            [this](const boost::system::error_code& error, std::size_t bytes_recvd)
            {
                if (!error)
                {
                    std::cout << "Received data: " << data_ << " from " << remote_endpoint_.address().to_string() << std::endl;
                    // Process the received data if needed

                    start_receive(); // Continue listening for incoming data
                }
            });
    }
    udp::socket socket_;
    udp::endpoint remote_endpoint_;
    enum { max_length = 1024 };
    char data_[max_length];
};
