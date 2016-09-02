/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
 * http://osgearth.org
 *
 * osgEarth is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include <osgEarth/GLSLChunker>
#include <osgEarth/StringUtils>
#include <sstream>

using namespace osgEarth;


void
GLSLChunker::dump(const std::string& msg, const Chunks& chunks) const
{
    OE_INFO << msg << "\n";
    for (Chunks::const_iterator i = chunks.begin(); i != chunks.end(); ++i)
    {
        std::string type = 
            i->type == Chunk::TYPE_DIRECTIVE ? "DIRECTIVE" :
            i->type == Chunk::TYPE_COMMENT ? "COMMENT" :
            i->type == Chunk::TYPE_STATEMENT ? "STATEMENT" :
            i->type == Chunk::TYPE_FUNCTION ? "FUNCTION" :
            "????????";

        OE_INFO << "   " << type << ": " << i->text << std::endl;
    }
}

#define ADVANCE \
    in >> std::noskipws >> c0; \
    c1 = in.peek()

#define SAVE \
    output.push_back(Chunk()); \
    output.back().text = chunk.str(); \
    std::string tok = token.str(); \
    if ( !tok.empty() ) tokens.push_back(tok); \
    output.back().tokens = tokens; \
    output.back().type = type; \
    chunk.str(std::string()); chunk.clear(); \
    token.str(std::string()); token.clear(); \
    tokens.clear(); \
    inChunk = false; \
    isBlockComment = false

#define TOKENIZE \
    if (::isspace(c0)) { \
        std::string tok = token.str(); \
        if (!tok.empty()) tokens.push_back(tok); \
        token.str(std::string()); token.clear(); \
    } \
    else token << c0


void
GLSLChunker::read(const std::string& input, Chunks& output) const
{
    char c0(0), c1(0);
    int braceLevel = 0;
    int parenLevel = 0;
    bool isBlockComment = false;

    // pad the input with a final newline so we always get the final line.
    std::string paddedInput(input);
    paddedInput.append("\n");

    std::istringstream in(paddedInput);
    std::ostringstream chunk;
    std::ostringstream token;
    std::vector<std::string> tokens;

    bool inChunk = false;
    Chunk::Type type;

    while (in)
    {
        ADVANCE;

        // are we inside a chunk?
        if (inChunk)
        {
            if (type == Chunk::TYPE_DIRECTIVE && c0 == '\n')
            {
                SAVE;
            }

            else if (type == Chunk::TYPE_STATEMENT && c0 == ';' && braceLevel == 0)
            {
                chunk << c0;
                TOKENIZE;
                SAVE;
            }

            else if (type == Chunk::TYPE_FUNCTION && c0 == '}' && braceLevel == 1)
            {
                --braceLevel;
                chunk << c0;
                TOKENIZE;
                SAVE;
            }

            else if (type == Chunk::TYPE_COMMENT && c0 == '\n' && !isBlockComment)
            {
                SAVE;
            }

            else if (type == Chunk::TYPE_COMMENT && c0 == '*' && c1 == '/' && isBlockComment)
            {
                chunk << c0;
                TOKENIZE;
                ADVANCE;
                chunk << c0;
                TOKENIZE;
                SAVE;
            }

            else {
                chunk << c0;
            }

            if (inChunk)
            {
                TOKENIZE;

                if (c0 == '(') {
                    ++parenLevel;
                }

                else if (c0 == ')') {
                    --parenLevel;
                }

                else if (c0 == '{') {
                    ++braceLevel;
                    // if we were in a statement, the precense of an open-brace converts it to a FUNCTION
                    // unless we can detect that it's a struct.
                    if (type == Chunk::TYPE_STATEMENT) {
                        if (tokens.empty() || tokens[0] != "struct") {
                            type = Chunk::TYPE_FUNCTION;
                        }
                    }
                }

                else if (c0 == '}') {
                    --braceLevel;
                }
            }
        }

        else // !inChunk
        {
            // not in a chunk; try to start a new one.

            if (c0 == '/' && c1 == '/') {
                type = Chunk::TYPE_COMMENT;
                isBlockComment = false;
                inChunk = true;
                chunk << c0;
                TOKENIZE;
            }
            else if (c0 == '/' && c1 == '*') {
                type = Chunk::TYPE_COMMENT;
                isBlockComment = true;
                inChunk = true;
                chunk << c0;
                TOKENIZE;
            }
            else if (c0 == '#') {
                type = Chunk::TYPE_DIRECTIVE;
                inChunk = true;
                chunk << c0;
                TOKENIZE;
            }
            else if (!::isspace(c0)) {
                type = Chunk::TYPE_STATEMENT;
                inChunk = true;
                chunk << c0;
                TOKENIZE;
            }
        }
    }
}


void
GLSLChunker::write(const Chunks& input, std::string& output) const
{
    std::stringstream buf;
    for(int i=0; i<input.size(); ++i)
    {
        buf << input[i].text << "\n";
    }
    output = buf.str();
}

GLSLChunker::Chunk
GLSLChunker::chunkLine(const std::string& line) const
{
    Chunks chunks;
    read(line, chunks);
    return chunks.size() > 0 ? chunks[0] : Chunk();
}

void
GLSLChunker::replace(Chunks& input, const std::string& pattern, const std::string& replacement) const
{
    for(int i=0; i<input.size(); ++i)
    {
        Chunk& chunk = input[i];
        osgEarth::replaceIn(chunk.text, pattern, replacement);
        for (unsigned t = 0; t<chunk.tokens.size(); ++t)
            osgEarth::replaceIn(chunk.tokens[t], pattern, replacement);
    }
}
