/**
 * Teseo Micromouse Virtual Competition
 * Main entry point
 *
 * Each .cpp file in src/agents/ is a mouse and builds its own executable.
 */

#include <cstdio>
#include <map>
#include <string>
#include <iostream>

#include "raylib.h"

#include "sim/maze.h"
#include "ui/ui.h"

/**
 * @brief Very simple command-line parser
 *
 * @param argc Argument count
 * @param argv Argument values
 *
 * @return A map of argument keys and values. Arguments should be in the form "--key value".
 */
std::map<std::string, std::string> ParseArgs(int argc, char *argv[])
{
    std::map<std::string, std::string> args;

    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];

        if (arg.substr(0, 2) == "--")
        {
            std::string key = arg.substr(2);

            if (i + 1 >= argc || std::string(argv[i + 1]).substr(0, 2) == "--")
            {
                std::cerr << "error: missing value for argument: " << arg << std::endl;

                continue;
            }

            args[key] = argv[++i];
        }
        else
            std::cerr << "error: unexpected argument: " << arg << std::endl;
    }

    return args;
}

/**
 * @brief Main entry point of the program.
 */
int main(int argc, char *argv[])
{
    // Parse command line arguments
    std::map<std::string, std::string> args = ParseArgs(argc, argv);
    Maze *maze = nullptr;

    if (args.contains("gen"))
    {
        uint32_t seed;

        try
        {
            seed = std::stoul(args["gen"]);
        }
        catch (const std::exception &)
        {
            std::cerr << "error: invalid seed: " << args["gen"] << std::endl;

            return 1;
        }

        std::cout << "Generating maze with seed " << seed << "...\n";

        maze = GenerateMaze(seed);
    }
    else if (args.contains("file"))
    {
        std::string filename = args["file"];

        maze = LoadMaze(filename.c_str());

        if (!maze)
        {
            std::cerr << "error: failed to load maze" << std::endl;

            return 1;
        }
    }
    else
    {
        printf("Usage: teseo [options]\n");
        printf("Options:\n");
        printf("  --gen <number>        Generate a random maze\n");
        printf("  --file <path>         Load the maze from a file\n");

        return 0;
    }

    // Create the UI
    CreateUI(maze);

    // Main loop
    while (UpdateUI())
        ;

    // Cleanup
    DestroyUI();
    DestroyMaze(maze);

    return 0;
}
