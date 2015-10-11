#ifndef MEMGRAPH_SERVER_HTTP_RESPONSE_INL
#define MEMGRAPH_SERVER_HTTP_RESPONSE_INL

#include "response.hpp"
#include "httpconnection.hpp"

#include "utils/memory/block_allocator.hpp"

namespace http
{

static BlockAllocator<sizeof(uv_write_t)> write_req_allocator;

template <class Req, class Res>
Response<Req, Res>::Response(connection_t& connection)
    : status(Status::Ok), connection(connection), buffer() {}

template <class Req, class Res>
void Response<Req, Res>::send(Status status, const std::string& body)
{
    this->status = status;
    this->send(body);
}

template <class Req, class Res>
void Response<Req, Res>::send(const std::string& body)
{
    uv_write_t* write_req =
        static_cast<uv_write_t*>(write_req_allocator.acquire());

    write_req->data = &connection;

    buffer << "HTTP/1.1 " << to_string[status] << "\r\n";

    buffer << "Content-Length:" << std::to_string(body.size()) << "\r\n";

    buffer << "Connection:" << (connection.keep_alive ? "Keep-Alive" : "Close")
           << "\r\n";

    for(auto it = headers.begin(); it != headers.end(); ++it)
        buffer << it->first << ":" << it->second << "\r\n";

    buffer << "\r\n" << body;

    uv_write(write_req, connection.client, buffer, buffer.count(), 
            [](uv_write_t* write_req, int) {

        connection_t& conn = *reinterpret_cast<connection_t*>(write_req->data);

        if(!conn.keep_alive)
            conn.close();

        conn.response.status = Status::Ok;
        conn.response.buffer.clear();
        conn.response.headers.clear();
        write_req_allocator.release(write_req);
    });
}

}

#endif
