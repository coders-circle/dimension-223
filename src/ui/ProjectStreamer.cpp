#include <stdinc.h>
#include <ui/ProjectStreamer.h>

#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <thread>


bool ProjectStreamer::mListening = false;
int ProjectStreamer::mSocket;
std::vector<int> ProjectStreamer::mClients;


inline std::vector<char> ReadAllBytes(char const* filename)
{
    std::ifstream ifs(filename, std::ios::binary|std::ios::ate);
    std::ifstream::pos_type pos = ifs.tellg();

    std::vector<char> result(pos);

    ifs.seekg(0, std::ios::beg);
    ifs.read(&result[0], pos);

    return result;
}


ProjectStreamer::ProjectStreamer(Gtk::Window* parent, Project& project)
    : mProject(project),
      mHBox(Gtk::ORIENTATION_HORIZONTAL), mStartBtn("Start")
{
    set_transient_for(*parent);
    get_content_area()->add(mHBox);

    if (mListening)
        mStartBtn.set_label("Stop");

    mStartBtn.signal_clicked().connect([&]() {
        if (mListening) {
            shutDown();
            return;
        }

        int csock;
        sockaddr_in sin;
        unsigned short port = 1234;

        if ((mSocket=socket(AF_INET, SOCK_STREAM, 0)) == -1) {
            std::cout << "Error while creating socket." << std::endl;
            return;
        }

        sin.sin_family = AF_INET;
        sin.sin_port = htons(port);
        sin.sin_addr.s_addr = htonl(INADDR_ANY);

        if (bind(mSocket, (sockaddr*)&sin, sizeof(sin)) != 0) {
            std::cout << "Bind error" << std::endl;
            return;
        }

        std::cout << "Listening" << std::endl;
        if (listen(mSocket, SOMAXCONN) != 0) {
            std::cout << "Listen error" << std::endl;
            return;
        }

        std::thread t1([&]() {
            std::cout << "Accepting" << std::endl;

            mListening = true;
            mStartBtn.set_label("Stop");

            while (true) {
                if ((csock = accept(mSocket, NULL, NULL)) == -1) {
                    std::cout << "Accept error" << std::endl;
                    ::close(mSocket);
                    return;
                }

                std::cout << "One Accepted" << std::endl;

                mClients.push_back(csock);
                streamProject(csock);
            }
        });
        t1.detach();
    });
    mHBox.add(mStartBtn);

    show_all();
}


void ProjectStreamer::shutDown() {
    if (mListening) {
        for (int client: mClients) {
            ::close(client);
        }

        shutdown(mSocket, SHUT_RDWR);
        ::close(mSocket);
        mListening = false;
        mStartBtn.set_label("Start");
    }
}


void ProjectStreamer::streamProject(int client) {
    int size = (int)mProject.getNumPointClouds();
    send(client, &size, sizeof(size), 0);
    for (size_t i=0; i<mProject.getNumPointClouds(); ++i) {
        auto& pc = mProject.getPointCloud(i);

        auto& vertices = pc.getVertices();
        auto& indices = pc.getIndices();

        size = (int)vertices.size();
        send(client, &size, sizeof(size), 0);
        std::cout << "Sending " << size << " vertices." << std::endl;
        send(client, &vertices[0], sizeof(float)*vertices.size(), 0);

        size = (int)indices.size();
        send(client, &size, sizeof(size), 0);
        std::cout << "Sending " << size << " indices." << std::endl;
        send(client, &indices[0], sizeof(int)*indices.size(), 0);


        cv::Mat& image = pc.getInputData().getImage();
        cv::imwrite("tmp.bmp", image);
        std::vector<char> iBytes = ReadAllBytes("tmp.bmp");

        size = (int)iBytes.size();
        send(client, &size, sizeof(size), 0);
        std::cout << "Sending " << size << " bytes of texture." << std::endl;
        send(client, &iBytes[0], size, 0);

        // TODO: Send in transformation.
    }

    // TODO: Send models.

    std::cout << "Sent all" << std::endl;
}
