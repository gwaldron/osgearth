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

#if defined(__EMSCRIPTEN__)
#define GLFW_INCLUDE_ES3
#include <GLFW/glfw3.h>
#include <GLES3/gl3.h>
#include <GLES3/gl2ext.h>
#else
#include <osg/GL>
//#include <glad.h>
#endif
#include <vector>
#include <string>
#include <iostream>
#include <unordered_map>
#include <osg/Vec2>
#include <osg/Vec3>
#include <osg/Vec4>
#include <osg/Matrixf>
#include <osg/GLExtensions>

namespace glm {
    using vec2 = osg::Vec2f;
    using vec3 = osg::Vec3f;
    using vec4 = osg::Vec4f;
    //using mat2 = osg::Matrix2f;
    //using mat3 = osg::Matrix3f;
    using mat4 = osg::Matrixf;
}
//#include <glm.hpp>
#undef DW_LOG_ERROR
#define DW_LOG_ERROR(X) std::cout << X << std::endl

#define DW_ENABLE_GL_ERROR_CHECK
// OpenGL error checking macro.
#ifdef DW_ENABLE_GL_ERROR_CHECK
#define GL_CHECK_ERROR(x)																		  \
x; {                                                                                              \
GLenum err(glGetError());																		  \
																								  \
while (err != GL_NO_ERROR)																		  \
{																								  \
std::string error;																				  \
																								  \
switch (err)																					  \
{																								  \
case GL_INVALID_OPERATION:      error = "INVALID_OPERATION";      break;						  \
case GL_INVALID_ENUM:           error = "INVALID_ENUM";           break;						  \
case GL_INVALID_VALUE:          error = "INVALID_VALUE";          break;						  \
case GL_OUT_OF_MEMORY:          error = "OUT_OF_MEMORY";          break;						  \
case 0x0506:  error = "INVALID_FRAMEBUFFER_OPERATION";  break;		  \
}																								  \
																								  \
std::string formatted_error = "OPENGL: ";														  \
formatted_error = formatted_error + error;														  \
formatted_error = formatted_error + ", LINE:";												      \
formatted_error = formatted_error + std::to_string(__LINE__);									  \
DW_LOG_ERROR(formatted_error);																	  \
err = glGetError();																				  \
}																								  \
}
#else
#define GL_CHECK_ERROR(x)	x
#endif

#if defined(__EMSCRIPTEN__)
#define GL_WRITE_ONLY 0
#endif

namespace dw
{
    extern osg::GLExtensions* ext();
    extern void setExt(osg::GLExtensions*);

	// Texture base class.
    class Texture
    {
    public:
        Texture();
        virtual ~Texture();

		// Bind texture to specified texture unit i.e GL_TEXTURE<unit>.
        void bind(uint32_t unit);
        void unbind(uint32_t unit);
        
        // Binding to image units.
        void bind_image(uint32_t unit, uint32_t mip_level, uint32_t layer, GLenum access, GLenum format);
        
        // Mipmap generation.
		void generate_mipmaps();
        
        // Getters.
		GLuint id();
		GLenum target();
		GLenum internal_format();
		GLenum format();
		GLenum type();
		uint32_t array_size();
        
        // Texture sampler functions.
        void set_wrapping(GLenum s, GLenum t, GLenum r);
        void set_border_color(float r, float g, float b, float a);
        void set_min_filter(GLenum filter);
        void set_mag_filter(GLenum filter);
		void set_compare_mode(GLenum mode);
		void set_compare_func(GLenum func);
        
    protected:
        GLuint m_gl_tex;
		GLenum m_target;
		GLenum m_internal_format;
		GLenum m_format; 
		GLenum m_type;
		uint32_t m_array_size;
    };
 
#if !defined(__EMSCRIPTEN__)
    class Texture1D : public Texture
    {
	public:
		Texture1D(uint32_t w, uint32_t array_size, int32_t mip_levels, GLenum internal_format, GLenum format, GLenum type);
		~Texture1D();
		void set_data(int array_index, int mip_level, void* data);
		uint32_t width();
		uint32_t mip_levels();

	private:
		uint32_t m_width;
		uint32_t m_mip_levels;
		
    };
#endif
    
    class Texture2D : public Texture
    {
    public:
		//static Texture2D* create_from_files(std::string path, bool srgb = true);
        Texture2D(uint32_t w, uint32_t h, uint32_t array_size, int32_t mip_levels, uint32_t num_samples, GLenum internal_format, GLenum format, GLenum type);
        ~Texture2D();
		void set_data(int array_index, int mip_level, void* data);
        uint32_t width();
        uint32_t height();
		uint32_t mip_levels();
		uint32_t num_samples();

	private:
		uint32_t m_width;
		uint32_t m_height;
		uint32_t m_mip_levels;
		uint32_t m_num_samples;
    };
    
    class Texture3D : public Texture
    {
	public:
		Texture3D(uint32_t w, uint32_t h, uint32_t d, int mip_levels, GLenum internal_format, GLenum format, GLenum type);
		~Texture3D();
		void set_data(int mip_level, void* data);
		uint32_t width();
		uint32_t height();
		uint32_t depth();
		uint32_t mip_levels();

	private:
		uint32_t m_width;
		uint32_t m_height;
		uint32_t m_depth;
		uint32_t m_mip_levels;
    };
    
    class TextureCube : public Texture
    {
	public:
		//static TextureCube* create_from_files(std::string path[], bool srgb = true);
		TextureCube(uint32_t w, uint32_t h, uint32_t array_size, int32_t mip_levels, GLenum internal_format, GLenum format, GLenum type);
		~TextureCube();
		void set_data(int face_index, int layer_index, int mip_level, void* data);
		uint32_t width();
		uint32_t height();
		uint32_t mip_levels();

	private:
		uint32_t m_width;
		uint32_t m_height;
		uint32_t m_mip_levels;
    };
    
    class Framebuffer
    {
    public:
        Framebuffer();
        ~Framebuffer();
		void bind();
		void unbind();

		// Attach entire texture or entire layer of a layered texture as a render target.
		void attach_render_target(uint32_t attachment, Texture* texture, uint32_t layer, uint32_t mip_level, bool draw = true, bool read = true);

		// Attach multiple render targets.
		void attach_multiple_render_targets(uint32_t attachment_count, Texture** texture);

		// Attach a given face from a cubemap or a specific layer of a cubemap array as a render target.
		void attach_render_target(uint32_t attachment, TextureCube* texture, uint32_t face, uint32_t layer, uint32_t mip_level, bool draw = true, bool read = true);

		// Attach entire texture or entire layer of a layered texture as a depth stencil target.
		void attach_depth_stencil_target(Texture* texture, uint32_t layer, uint32_t mip_level);

		// Attach a given face from a cubemap or a specific layer of a cubemap array as a depth stencil target.
		void attach_depth_stencil_target(TextureCube* texture, uint32_t face, uint32_t layer, uint32_t mip_level);
        
		uint32_t render_targets();

    private:
        void check_status();

    private:
		uint32_t m_render_target_count = 0;
		GLuint m_attachments[16];
        GLuint m_gl_fbo;
    };
    
    class Shader
    {
        friend class Program;
        
    public:
        static Shader* create_from_file(GLenum type, std::string path, std::vector<std::string> defines = std::vector<std::string>());
        
        Shader(GLenum type, std::string source);
        ~Shader();
        GLenum type();
		bool compiled();
        
    private:
		bool m_compiled;
        GLuint m_gl_shader;
        GLenum m_type;
    };
    
    class Program
    {
    public:
        Program(uint32_t count, Shader** shaders);
        ~Program();
        void use();
		void uniform_block_binding(std::string name, int binding);
		bool set_uniform(std::string name, int value);
		bool set_uniform(std::string name, float value);
		bool set_uniform(std::string name, glm::vec2 value);
		bool set_uniform(std::string name, glm::vec3 value);
		bool set_uniform(std::string name, glm::vec4 value);
		//bool set_uniform(std::string name, glm::mat2 value);
		//bool set_uniform(std::string name, glm::mat3 value);
		bool set_uniform(std::string name, glm::mat4 value);
		bool set_uniform(std::string name, int count, int* value);
		bool set_uniform(std::string name, int count, float* value);
		bool set_uniform(std::string name, int count, glm::vec2* value);
		bool set_uniform(std::string name, int count, glm::vec3* value);
		bool set_uniform(std::string name, int count, glm::vec4* value);
		//bool set_uniform(std::string name, int count, glm::mat2* value);
		//bool set_uniform(std::string name, int count, glm::mat3* value);
		bool set_uniform(std::string name, int count, glm::mat4* value);
        
    private:
        GLuint m_gl_program;
		std::unordered_map<std::string, GLuint> m_location_map;
    };

	class Buffer
	{
	public:
		Buffer(GLenum type, GLenum usage, size_t size, void* data);
		virtual ~Buffer();
		void bind();
		void bind_base(int index);
		void bind_range(int index, size_t offset, size_t size);
		void unbind();
		void* map(GLenum access);
		void* map_range(GLenum access, size_t offset, size_t size);
		void unmap();
		void set_data(size_t offset, size_t size, void* data);

	protected:
		GLenum m_type;
		GLuint m_gl_buffer;
		size_t m_size;
#if defined(__EMSCRIPTEN__)
		void* m_staging;
		size_t m_mapped_size;
		size_t m_mapped_offset;
#endif
	};
    
    class VertexBuffer : public Buffer
    {
    public:
		VertexBuffer(GLenum usage, size_t size, void* data = nullptr);
        ~VertexBuffer();
    };
    
    class IndexBuffer : public Buffer
    {
    public:
		IndexBuffer(GLenum usage, size_t size, void* data = nullptr);
        ~IndexBuffer();
    };

	class UniformBuffer : public Buffer
	{
	public:
		UniformBuffer(GLenum usage, size_t size, void* data = nullptr);
		~UniformBuffer();
	};

#if !defined(__EMSCRIPTEN__)
	class ShaderStorageBuffer : public Buffer
	{
	public:
		ShaderStorageBuffer(GLenum usage, size_t size, void* data = nullptr);
		~ShaderStorageBuffer();
	};
#endif

	struct VertexAttrib
	{
		uint32_t num_sub_elements;
		uint32_t type;
		bool	 normalized;
		uint32_t offset;
	};
    
    class VertexArray
    {
    public:
		VertexArray(VertexBuffer* vbo, IndexBuffer* ibo, size_t vertex_size, int attrib_count, VertexAttrib attribs[]);
        ~VertexArray();
		void bind();
		void unbind();
        
    private:
        GLuint m_gl_vao;
    };

	class Query
	{
	public:
		Query();
		~Query();
		void query_counter(GLenum type);
		void begin(GLenum type);
		void end(GLenum type);
		void result_64(uint64_t* ptr);
		bool result_available();

	private:
		GLuint m_query;
	};
}
