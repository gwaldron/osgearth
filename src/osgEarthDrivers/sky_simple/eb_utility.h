#pragma once
/*
https://github.com/diharaw/BrunetonSkyModel

Copyright (c) 2018 Dihara Wijetunga

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#include <string>
#include <vector>
#include <sstream>
#include <cassert>
#include <algorithm>
#include <stdio.h>

namespace dw
{
	class Shader;
	class Program;

	namespace utility
	{
		// Returns the absolute path to the resource. It also resolves the path to the 'Resources' directory is macOS app bundles.
		extern std::string path_for_resource(const std::string& resource);

		// Returns the absolute path of the executable.
		extern std::string executable_path();

		// Reads the contents of a text file into an std::string. Returns false if file does not exist.
		extern bool read_text(std::string path, std::string& out);

		// Reads the specified shader source.
		extern bool read_shader(const std::string& path, std::string& out, std::vector<std::string> defines = std::vector<std::string>());

		extern bool preprocess_shader(const std::string& path, const std::string& src, std::string& out);

		// Removes the filename from a file path.
		extern std::string path_without_file(std::string filepath);

		// Returns the extension of a given file.
		extern std::string file_extension(std::string filepath);

		extern std::string file_name_from_path(std::string filepath);

		// Queries the current working directory.
		extern std::string current_working_directory();

		// Changes the current working directory.
		extern void change_current_working_directory(std::string path);

		// Create compute program
		extern bool create_compute_program(const std::string& path, Shader** shader, Program** program, std::vector<std::string> defines = std::vector<std::string>());

        extern bool create_compute_program_from_string(const std::string& path, Shader** shader, Program** program, std::vector<std::string> defines = std::vector<std::string>());
	} // namespace utility
} // namespace dw
