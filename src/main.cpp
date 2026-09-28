/**
 * Teseo Micromouse Virtual Competition
 * Main entry point
 *
 * Each .cpp file in src/agents/ is a mouse and builds its own executable.
 */

#include <cstdio>
#include <map>
#include <random>
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
        // Program name without directory or extension: each mouse is its own executable
        std::string program = argv[0];
        program = program.substr(program.find_last_of("/\\") + 1);
        program = program.substr(0, program.find_last_of('.'));

        printf("Usage: %s [options]\n", program.c_str());
        printf("Options:\n");
        printf("  --gen <number>        Generate a random maze\n");
        printf("  --file <path>         Load the maze from a file\n");
        printf("  --noise-seed <number> Repeat the random errors of a previous execution\n");

        return 0;
    }

    // Noise seed: random unless given, and always printed, so that any execution can be repeated
    uint32_t noise_seed = std::random_device{}();
    if (args.contains("noise-seed"))
    {
        try
        {
            noise_seed = std::stoul(args["noise-seed"]);
        }
        catch (const std::exception &)
        {
            std::cerr << "error: invalid noise seed: " << args["noise-seed"] << std::endl;

            return 1;
        }
    }

    std::cout << "Noise seed: " << noise_seed << " (repeat this execution with --noise-seed " << noise_seed << ")\n";

    // Create the UI
    CreateUI(maze, noise_seed);

    // Main loop
    while (UpdateUI())
        ;

    // Cleanup
    DestroyUI();
    DestroyMaze(maze);

    return 0;
}
