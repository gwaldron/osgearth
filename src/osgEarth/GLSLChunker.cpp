/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "GLSLChunker"
#include "StringUtils"
#include "Notify"

using namespace osgEarth;
using namespace osgEarth::Util;

#define LC "[GLSLChunker] "

void
GLSLChunker::dump(const std::string& msg, const Chunks& chunks) const
{
    OE_DEBUG << LC << msg << std::endl;
    for (Chunks::const_iterator i = chunks.begin(); i != chunks.end(); ++i)
    {
        std::string type = 
            i->type == Chunk::TYPE_DIRECTIVE ? "DIRECTIVE" :
            i->type == Chunk::TYPE_COMMENT ? "COMMENT" :
            i->type == Chunk::TYPE_STATEMENT ? "STATEMENT" :
            i->type == Chunk::TYPE_FUNCTION ? "FUNCTION" :
            "????????";

        OE_DEBUG << "   " << type << ": " << i->text << std::endl;
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

                else if (c0 == '{')
                {
                    ++braceLevel;
                    // if we were in a statement, the precense of an open-brace converts it to a FUNCTION
                    // unless we can detect that it's a struct or an interface block.
                    if (type == Chunk::TYPE_STATEMENT)
                    {
                        if (tokens.empty() || 
                            (tokens[0] != "struct"  && 
                             tokens[0] != "in"      &&
                             tokens[0] != "out"     &&
                             tokens[0].substr(0,6) != "layout"))
                        {
                            type = Chunk::TYPE_FUNCTION;
                        }
                    }
                }

                else if (c0 == '}')
                {
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

namespace
{
    std::string indent(const std::string& input)
    {
        std::ostringstream out;

        auto lines = StringTokenizer()
            .delim("\n")
            .keepEmpties(true)
            .tokenize(input);

        std::string prefix;
        for (auto& line : lines)
        {
            auto temp = Strings::trim(line);
            auto pos = temp.find_first_not_of(" \t", 0);
            if (pos > 0 && pos != temp.npos)
                temp = temp.substr(pos);

            bool is_directive = (temp.size() > 0 && temp[0] == '#');

            if (temp.find('}') != temp.npos && prefix.size() >= 4)
                prefix.resize(prefix.size() - 4);

            if (is_directive)
                out << temp << "\n";
            else
                out << prefix << temp << "\n";
            
            if (temp.find('{') != temp.npos)
                prefix += "    ";
        }
        return out.str();
    }
}

void
GLSLChunker::write(const Chunks& input, std::string& output, bool reindent) const
{
    std::stringstream buf;
    for(int i=0; i<input.size(); ++i)
    {
        if (reindent && input[i].type != Chunk::TYPE_COMMENT && input[i].type != Chunk::TYPE_DIRECTIVE)
        {
            buf << indent(input[i].text);
        }
        else
        {
            buf << input[i].text << "\n";
        }
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
        osgEarth::Util::replaceIn(chunk.text, pattern, replacement);
        for (unsigned t = 0; t<chunk.tokens.size(); ++t)
            osgEarth::Util::replaceIn(chunk.tokens[t], pattern, replacement);
    }
}

