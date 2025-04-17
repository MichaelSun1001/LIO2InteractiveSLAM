#include "../odometry2graph/include/odometry2graph.h"

int main(int argc, char **argv)
{
    odometry2graph *app = new odometry2graph();
    if (app->load_config(std::string(argv[1])))
    {
        app->process();
        app->save_map();
    }

    return 0;
}