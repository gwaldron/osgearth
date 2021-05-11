/* -*-c++-*- */
/* osgEarth - Geospatial SDK for OpenSceneGraph
 * Copyright 2020 Pelican Mapping
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

#include <osgEarth/GL>
#include <osg/GLExtensions>
#include <osg/ref_ptr>

static osg::ref_ptr< osg::GLExtensions > s_glext;

void osgEarth::initGLExtensions(unsigned int contextID)
{
    s_glext = osg::GLExtensions::Get(contextID, true);
}

// GL_VERSION_1_2
void glDrawRangeElements(GLenum mode, GLuint start, GLuint end, GLsizei count, GLenum type, const void *indices)
{
    s_glext->glDrawRangeElements(mode, start, end, count, type, indices);
}

void glTexImage3D(GLenum target, GLint level, GLint internalformat, GLsizei width, GLsizei height, GLsizei depth, GLint border, GLenum format, GLenum type, const void *pixels)
{
    s_glext->glTexImage3D(target, level, internalformat, width, height, depth, border, format, type, pixels);
}

void glTexSubImage3D(GLenum target, GLint level, GLint xoffset, GLint yoffset, GLint zoffset, GLsizei width, GLsizei height, GLsizei depth, GLenum format, GLenum type, const void *pixels)
{
    s_glext->glTexSubImage3D(target, level, xoffset, yoffset, zoffset, width, height, depth, format, type, pixels);
}

void glCopyTexSubImage3D(GLenum target, GLint level, GLint xoffset, GLint yoffset, GLint zoffset, GLint x, GLint y, GLsizei width, GLsizei height)
{
    s_glext->glCopyTexSubImage3D(target, level, xoffset, yoffset, zoffset, x, y, width, height);
}


// GL_VERSION_1_3
void glActiveTexture(GLenum texture)
{
    s_glext->glActiveTexture(texture);
}

void glSampleCoverage(GLfloat value, GLboolean invert)
{
    s_glext->glSampleCoverage(value, invert);
}

void glCompressedTexImage3D(GLenum target, GLint level, GLenum internalformat, GLsizei width, GLsizei height, GLsizei depth, GLint border, GLsizei imageSize, const void *data)
{
    s_glext->glCompressedTexImage3D(target, level, internalformat, width, height, depth, border, imageSize, data);
}

void glCompressedTexImage2D(GLenum target, GLint level, GLenum internalformat, GLsizei width, GLsizei height, GLint border, GLsizei imageSize, const void *data)
{
    s_glext->glCompressedTexImage2D(target, level, internalformat, width, height, border, imageSize, data);
}

void glCompressedTexImage1D(GLenum target, GLint level, GLenum internalformat, GLsizei width, GLint border, GLsizei imageSize, const void *data)
{
    // TODO:
    //s_glext->glCompressedTexImage1D(target, level, internalformat, width, border, imageSize, data);
}

void glCompressedTexSubImage3D(GLenum target, GLint level, GLint xoffset, GLint yoffset, GLint zoffset, GLsizei width, GLsizei height, GLsizei depth, GLenum format, GLsizei imageSize, const void *data)
{
    s_glext->glCompressedTexSubImage3D(target, level, xoffset, yoffset, zoffset, width, height, depth, format, imageSize, data);
}

void glCompressedTexSubImage2D(GLenum target, GLint level, GLint xoffset, GLint yoffset, GLsizei width, GLsizei height, GLenum format, GLsizei imageSize, const void *data)
{
    s_glext->glCompressedTexSubImage2D(target, level, xoffset, yoffset, width, height, format, imageSize, data);
}

void glCompressedTexSubImage1D(GLenum target, GLint level, GLint xoffset, GLsizei width, GLenum format, GLsizei imageSize, const void *data)
{
    // TODO:
    // s_glext->glCompressedTexSubImage1D(target, level, xoffset, width, format, imageSize, data);
}

void glGetCompressedTexImage(GLenum target, GLint level, void *img)
{
    s_glext->glGetCompressedTexImage(target, level, img);
}


// GL_VERSION_1_4
void glBlendFuncSeparate(GLenum sfactorRGB, GLenum dfactorRGB, GLenum sfactorAlpha, GLenum dfactorAlpha)
{
    s_glext->glBlendFuncSeparate(sfactorRGB, dfactorRGB, sfactorAlpha, dfactorAlpha);
}

void glMultiDrawArrays(GLenum mode, const GLint *first, const GLsizei *count, GLsizei drawcount)
{
    s_glext->glMultiDrawArrays(mode, first, count, drawcount);
}

void glMultiDrawElements(GLenum mode, const GLsizei *count, GLenum type, const void *const*indices, GLsizei drawcount)
{
    s_glext->glMultiDrawElements(mode, count, type, indices, drawcount);
}

void glPointParameterf(GLenum pname, GLfloat param)
{
    s_glext->glPointParameterf(pname, param);
}

void glPointParameterfv(GLenum pname, const GLfloat *params)
{
    s_glext->glPointParameterfv(pname, params);
}

void glPointParameteri(GLenum pname, GLint param)
{
    s_glext->glPointParameteri(pname, param);
}

void glPointParameteriv(GLenum pname, const GLint *params)
{
    // TODO:
    //s_glext->glPointParameteriv(pname, params);
}

void glBlendColor(GLfloat red, GLfloat green, GLfloat blue, GLfloat alpha)
{
    s_glext->glBlendColor(red, green, blue, alpha);
}

void glBlendEquation(GLenum mode)
{
    s_glext->glBlendEquation(mode);
}

// GL_VERSION_1_5
void glGenQueries(GLsizei n, GLuint *ids)
{
    s_glext->glGenQueries(n, ids);
}

void glDeleteQueries(GLsizei n, const GLuint *ids)
{
    s_glext->glDeleteQueries(n, ids);
}

GLboolean glIsQuery(GLuint id)
{
    return s_glext->glIsQuery(id);
}

void glBeginQuery(GLenum target, GLuint id)
{
    s_glext->glBeginQuery(target, id);
}

void glEndQuery(GLenum target)
{
    s_glext->glEndQuery(target);
}

void glGetQueryiv(GLenum target, GLenum pname, GLint *params)
{
    s_glext->glGetQueryiv(target, pname, params);
}

void glGetQueryObjectiv(GLuint id, GLenum pname, GLint *params)
{
    s_glext->glGetQueryObjectiv(id, pname, params);
}

void glGetQueryObjectuiv(GLuint id, GLenum pname, GLuint *params)
{
    s_glext->glGetQueryObjectuiv(id, pname, params);
}

void glBindBuffer(GLenum target, GLuint buffer)
{
    s_glext->glBindBuffer(target, buffer);
}

void glDeleteBuffers(GLsizei n, const GLuint *buffers)
{
    s_glext->glDeleteBuffers(n, buffers);
}

void glGenBuffers(GLsizei n, GLuint *buffers)
{
    s_glext->glGenBuffers(n, buffers);
}

GLboolean glIsBuffer(GLuint buffer)
{
    return s_glext->glIsBuffer(buffer);
}

void glBufferData(GLenum target, GLsizeiptr size, const void *data, GLenum usage)
{
    s_glext->glBufferData(target, size, data, usage);
}

void glBufferSubData(GLenum target, GLintptr offset, GLsizeiptr size, const void *data)
{
    s_glext->glBufferSubData(target, offset, size, data);
}

void glGetBufferSubData(GLenum target, GLintptr offset, GLsizeiptr size, void *data)
{
    s_glext->glGetBufferSubData(target, offset, size, data);
}

void *glMapBuffer(GLenum target, GLenum access)
{
    return s_glext->glMapBuffer(target, access);
}

GLboolean glUnmapBuffer(GLenum target)
{
    return s_glext->glUnmapBuffer(target);
}

void glGetBufferParameteriv(GLenum target, GLenum pname, GLint *params)
{
    s_glext->glGetBufferParameteriv(target, pname, params);
}

void glGetBufferPointerv(GLenum target, GLenum pname, void **params)
{
    s_glext->glGetBufferPointerv(target, pname, params);
}


// GL_VERSION_2_0
void glBlendEquationSeparate(GLenum modeRGB, GLenum modeAlpha)
{
    s_glext->glBlendEquationSeparate(modeRGB, modeAlpha);
}

void  glDrawBuffers(GLsizei n, const GLenum *bufs)
{
    s_glext->glDrawBuffers(n, bufs);
}

void  glStencilOpSeparate(GLenum face, GLenum sfail, GLenum dpfail, GLenum dppass)
{
    s_glext->glStencilOpSeparate(face, sfail, dpfail, dppass);
}

void  glStencilFuncSeparate(GLenum face, GLenum func, GLint ref, GLuint mask)
{
    s_glext->glStencilFuncSeparate(face, func, ref, mask);
}

void  glStencilMaskSeparate(GLenum face, GLuint mask)
{
    s_glext->glStencilMaskSeparate(face, mask);
}

void  glAttachShader(GLuint program, GLuint shader)
{
    s_glext->glAttachShader(program, shader);
}

void  glBindAttribLocation(GLuint program, GLuint index, const GLchar *name)
{
    s_glext->glBindAttribLocation(program, index, name);
}

void  glCompileShader(GLuint shader)
{
    s_glext->glCompileShader(shader);
}

GLuint  glCreateProgram(void)
{
    return s_glext->glCreateProgram();
}

GLuint  glCreateShader(GLenum type)
{
    return s_glext->glCreateShader(type);
}

void  glDeleteProgram(GLuint program)
{
    s_glext->glDeleteProgram(program);
}

void  glDeleteShader(GLuint shader)
{
    s_glext->glDeleteShader(shader);
}

void  glDetachShader(GLuint program, GLuint shader)
{
    s_glext->glDetachShader(program, shader);
}

void  glDisableVertexAttribArray(GLuint index)
{
    s_glext->glDisableVertexAttribArray(index);
}

void glEnableVertexAttribArray(GLuint index)
{
    s_glext->glEnableVertexAttribArray(index);
}

void  glGetActiveAttrib(GLuint program, GLuint index, GLsizei bufSize, GLsizei *length, GLint *size, GLenum *type, GLchar *name)
{
    s_glext->glGetActiveAttrib(program, index, bufSize, length, size, type, name);
}

void  glGetActiveUniform(GLuint program, GLuint index, GLsizei bufSize, GLsizei *length, GLint *size, GLenum *type, GLchar *name)
{
    s_glext->glGetActiveUniform(program, index, bufSize, length, size, type, name);
}

void  glGetAttachedShaders(GLuint program, GLsizei maxCount, GLsizei *count, GLuint *shaders)
{
    s_glext->glGetAttachedShaders(program, maxCount, count, shaders);
}

GLint  glGetAttribLocation(GLuint program, const GLchar *name)
{
    return s_glext->glGetAttribLocation(program, name);
}

void  glGetProgramiv(GLuint program, GLenum pname, GLint *params)
{
    s_glext->glGetProgramiv(program, pname, params);
}

void  glGetProgramInfoLog(GLuint program, GLsizei bufSize, GLsizei *length, GLchar *infoLog)
{
    s_glext->glGetProgramInfoLog(program, bufSize, length, infoLog);
}

void  glGetShaderiv(GLuint shader, GLenum pname, GLint *params)
{
    s_glext->glGetShaderiv(shader, pname, params);
}

void  glGetShaderInfoLog(GLuint shader, GLsizei bufSize, GLsizei *length, GLchar *infoLog)
{
    s_glext->glGetShaderInfoLog(shader, bufSize, length, infoLog);
}

void  glGetShaderSource(GLuint shader, GLsizei bufSize, GLsizei *length, GLchar *source)
{
    s_glext->glGetShaderSource(shader, bufSize, length, source);
}

GLint  glGetUniformLocation(GLuint program, const GLchar *name)
{
    return s_glext->glGetUniformLocation(program, name);
}

void  glGetUniformfv(GLuint program, GLint location, GLfloat *params)
{
    s_glext->glGetUniformfv(program, location, params);
}

void  glGetUniformiv(GLuint program, GLint location, GLint *params)
{
    s_glext->glGetUniformiv(program, location, params);
}

void  glGetVertexAttribdv(GLuint index, GLenum pname, GLdouble *params)
{
    s_glext->glGetVertexAttribdv(index, pname, params);
}

void  glGetVertexAttribfv(GLuint index, GLenum pname, GLfloat *params)
{
    s_glext->glGetVertexAttribfv(index, pname, params);
}

void  glGetVertexAttribiv(GLuint index, GLenum pname, GLint *params)
{
    s_glext->glGetVertexAttribiv(index, pname, params);
}

void  glGetVertexAttribPointerv(GLuint index, GLenum pname, void **pointer)
{
    s_glext->glGetVertexAttribPointerv(index, pname, pointer);
}

GLboolean  glIsProgram(GLuint program)
{
    return s_glext->glIsProgram(program);
}

GLboolean  glIsShader(GLuint shader)
{
    return s_glext->glIsShader(shader);
}

void  glLinkProgram(GLuint program)
{
    s_glext->glLinkProgram(program);
}

void  glShaderSource(GLuint shader, GLsizei count, const GLchar *const*string, const GLint *length)
{
    // TODO:  Is this correct?
    const GLchar** str = const_cast<const GLchar**>(string);
    s_glext->glShaderSource(shader, count, str, length);
}

void  glUseProgram(GLuint program)
{
    s_glext->glUseProgram(program);
}

void  glUniform1f(GLint location, GLfloat v0)
{
    s_glext->glUniform1f(location, v0);
}

void  glUniform2f(GLint location, GLfloat v0, GLfloat v1)
{
    s_glext->glUniform2f(location, v0, v1);
}

void glUniform3f(GLint location, GLfloat v0, GLfloat v1, GLfloat v2)
{
    s_glext->glUniform3f(location, v0, v1, v2);
}

void  glUniform4f(GLint location, GLfloat v0, GLfloat v1, GLfloat v2, GLfloat v3)
{
    s_glext->glUniform4f(location, v0, v1, v2, v3);
}

void  glUniform1i(GLint location, GLint v0)
{
    s_glext->glUniform1i(location, v0);
}

void  glUniform2i(GLint location, GLint v0, GLint v1)
{
    s_glext->glUniform2i(location, v0, v1);
}

void  glUniform3i(GLint location, GLint v0, GLint v1, GLint v2)
{
    s_glext->glUniform3i(location, v0, v1, v2);
}

void  glUniform4i(GLint location, GLint v0, GLint v1, GLint v2, GLint v3)
{
    s_glext->glUniform4i(location, v0, v1, v2, v3);
}

void  glUniform1fv(GLint location, GLsizei count, const GLfloat *value)
{
    s_glext->glUniform1fv(location, count, value);
}

void  glUniform2fv(GLint location, GLsizei count, const GLfloat *value)
{
    s_glext->glUniform2fv(location, count, value);
}

void  glUniform3fv(GLint location, GLsizei count, const GLfloat *value)
{
    s_glext->glUniform3fv(location, count, value);
}

void  glUniform4fv(GLint location, GLsizei count, const GLfloat *value)
{
    s_glext->glUniform4fv(location, count, value);
}

void  glUniform1iv(GLint location, GLsizei count, const GLint *value)
{
    s_glext->glUniform1iv(location, count, value);
}

void  glUniform2iv(GLint location, GLsizei count, const GLint *value)
{
    s_glext->glUniform2iv(location, count, value);
}

void  glUniform3iv(GLint location, GLsizei count, const GLint *value)
{
    s_glext->glUniform3iv(location, count, value);
}

void  glUniform4iv(GLint location, GLsizei count, const GLint *value)
{
    s_glext->glUniform4iv(location, count, value);
}

void  glUniformMatrix2fv(GLint location, GLsizei count, GLboolean transpose, const GLfloat *value)
{
    s_glext->glUniformMatrix2fv(location, count, transpose, value);
}

void  glUniformMatrix3fv(GLint location, GLsizei count, GLboolean transpose, const GLfloat *value)
{
    s_glext->glUniformMatrix3fv(location, count, transpose, value);
}

void  glUniformMatrix4fv(GLint location, GLsizei count, GLboolean transpose, const GLfloat *value)
{
    s_glext->glUniformMatrix4fv(location, count, transpose, value);
}

void  glValidateProgram(GLuint program)
{
    s_glext->glValidateProgram(program);
}

void  glVertexAttrib1d(GLuint index, GLdouble x)
{
    s_glext->glVertexAttrib1d(index, x);
}

void  glVertexAttrib1dv(GLuint index, const GLdouble *v)
{
    s_glext->glVertexAttrib1dv(index, v);
}

void  glVertexAttrib1f(GLuint index, GLfloat x)
{
    s_glext->glVertexAttrib1f(index, x);
}

void  glVertexAttrib1fv(GLuint index, const GLfloat *v)
{
    s_glext->glVertexAttrib1fv(index, v);
}

void  glVertexAttrib1s(GLuint index, GLshort x)
{
    s_glext->glVertexAttrib1s(index, x);
}

void  glVertexAttrib1sv(GLuint index, const GLshort *v)
{
    s_glext->glVertexAttrib1sv(index, v);
}

void  glVertexAttrib2d(GLuint index, GLdouble x, GLdouble y)
{
    s_glext->glVertexAttrib2d(index, x, y);
}

void  glVertexAttrib2dv(GLuint index, const GLdouble *v)
{
    s_glext->glVertexAttrib2dv(index, v);
}

void  glVertexAttrib2f(GLuint index, GLfloat x, GLfloat y)
{
    s_glext->glVertexAttrib2f(index, x, y);
}

void  glVertexAttrib2fv(GLuint index, const GLfloat *v)
{
    s_glext->glVertexAttrib2fv(index, v);
}

void  glVertexAttrib2s(GLuint index, GLshort x, GLshort y)
{
    s_glext->glVertexAttrib2s(index, x, y);
}

void  glVertexAttrib2sv(GLuint index, const GLshort *v)
{
    s_glext->glVertexAttrib2sv(index, v);
}

void  glVertexAttrib3d(GLuint index, GLdouble x, GLdouble y, GLdouble z)
{
    s_glext->glVertexAttrib3d(index, x, y, z);
}

void  glVertexAttrib3dv(GLuint index, const GLdouble *v)
{
    s_glext->glVertexAttrib3dv(index, v);
}

void  glVertexAttrib3f(GLuint index, GLfloat x, GLfloat y, GLfloat z)
{
    s_glext->glVertexAttrib3f(index, x, y, z);
}

void  glVertexAttrib3fv(GLuint index, const GLfloat *v)
{
    s_glext->glVertexAttrib3fv(index, v);
}

void  glVertexAttrib3s(GLuint index, GLshort x, GLshort y, GLshort z)
{
    s_glext->glVertexAttrib3s(index, x, y, z);
}

void  glVertexAttrib3sv(GLuint index, const GLshort *v)
{
    s_glext->glVertexAttrib3sv(index, v);
}

void  glVertexAttrib4Nbv(GLuint index, const GLbyte *v)
{
    s_glext->glVertexAttrib4Nbv(index, v);
}

void  glVertexAttrib4Niv(GLuint index, const GLint *v)
{
    s_glext->glVertexAttrib4Niv(index, v);
}

void  glVertexAttrib4Nsv(GLuint index, const GLshort *v)
{
    s_glext->glVertexAttrib4Nsv(index, v);
}

void  glVertexAttrib4Nub(GLuint index, GLubyte x, GLubyte y, GLubyte z, GLubyte w)
{
    s_glext->glVertexAttrib4Nub(index, x, y, z, w);
}

void  glVertexAttrib4Nubv(GLuint index, const GLubyte *v)
{
    s_glext->glVertexAttrib4Nubv(index, v);
}

void  glVertexAttrib4Nuiv(GLuint index, const GLuint *v)
{
    s_glext->glVertexAttrib4Nuiv(index, v);
}

void  glVertexAttrib4Nusv(GLuint index, const GLushort *v)
{
    s_glext->glVertexAttrib4Nusv(index, v);
}

void  glVertexAttrib4bv(GLuint index, const GLbyte *v)
{
    s_glext->glVertexAttrib4bv(index, v);
}

void  glVertexAttrib4d(GLuint index, GLdouble x, GLdouble y, GLdouble z, GLdouble w)
{
    s_glext->glVertexAttrib4d(index, x, y, z, w);
}

void  glVertexAttrib4dv(GLuint index, const GLdouble *v)
{
    s_glext->glVertexAttrib4dv(index, v);
}

void  glVertexAttrib4f(GLuint index, GLfloat x, GLfloat y, GLfloat z, GLfloat w)
{
    s_glext->glVertexAttrib4f(index, x, y, z, w);
}

void  glVertexAttrib4fv(GLuint index, const GLfloat *v)
{
    s_glext->glVertexAttrib4fv(index, v);
}

void  glVertexAttrib4iv(GLuint index, const GLint *v)
{
    s_glext->glVertexAttrib4iv(index, v);
}

void  glVertexAttrib4s(GLuint index, GLshort x, GLshort y, GLshort z, GLshort w)
{
    s_glext->glVertexAttrib4s(index, x, y, z, w);
}

void  glVertexAttrib4sv(GLuint index, const GLshort *v)
{
    s_glext->glVertexAttrib4sv(index, v);
}

void  glVertexAttrib4ubv(GLuint index, const GLubyte *v)
{
    s_glext->glVertexAttrib4ubv(index, v);
}

void  glVertexAttrib4uiv(GLuint index, const GLuint *v)
{
    s_glext->glVertexAttrib4uiv(index, v);
}

void  glVertexAttrib4usv(GLuint index, const GLushort *v)
{
    s_glext->glVertexAttrib4usv(index, v);
}

void  glVertexAttribPointer(GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const void *pointer)
{
    s_glext->glVertexAttribPointer(index, size, type, normalized, stride, pointer);
}


// GL_VERSION_3_2
void glDrawElementsBaseVertex(GLenum mode, GLsizei count, GLenum type, const void *indices, GLint basevertex)
{
    s_glext->glDrawElementsBaseVertex(mode, count, type, indices, basevertex);
}

void glDrawRangeElementsBaseVertex(GLenum mode, GLuint start, GLuint end, GLsizei count, GLenum type, const void *indices, GLint basevertex)
{
    s_glext->glDrawRangeElementsBaseVertex(mode, start, end, count, type, indices, basevertex);
}

void glDrawElementsInstancedBaseVertex(GLenum mode, GLsizei count, GLenum type, const void *indices, GLsizei instancecount, GLint basevertex)
{
    s_glext->glDrawElementsInstancedBaseVertex(mode, count, type, indices, instancecount, basevertex);
}

void glMultiDrawElementsBaseVertex(GLenum mode, const GLsizei *count, GLenum type, const void *const*indices, GLsizei drawcount, const GLint *basevertex)
{
    s_glext->glMultiDrawElementsBaseVertex(mode, count, type, indices, drawcount, basevertex);
}

void glProvokingVertex(GLenum mode)
{
    s_glext->glProvokingVertex(mode);
}

GLsync glFenceSync(GLenum condition, GLbitfield flags)
{
    return s_glext->glFenceSync(condition, flags);
}

GLboolean glIsSync(GLsync sync)
{
    return s_glext->glIsSync(sync);
}

void glDeleteSync(GLsync sync)
{
    s_glext->glDeleteSync(sync);
}

GLenum glClientWaitSync(GLsync sync, GLbitfield flags, GLuint64 timeout)
{
    return s_glext->glClientWaitSync(sync, flags, timeout);
}

void glWaitSync(GLsync sync, GLbitfield flags, GLuint64 timeout)
{
    s_glext->glWaitSync(sync, flags, timeout);
}

void glGetInteger64v(GLenum pname, GLint64 *data)
{
    s_glext->glGetInteger64v(pname, data);
}

void glGetSynciv(GLsync sync, GLenum pname, GLsizei count, GLsizei *length, GLint *values)
{
    s_glext->glGetSynciv(sync, pname, count, length, values);
}

void glGetInteger64i_v(GLenum target, GLuint index, GLint64 *data)
{
    // TODO:

    // s_glext->glGetInteger64i_v(target, index, data);
}

void glGetBufferParameteri64v(GLenum target, GLenum pname, GLint64 *params)
{
    // TODO:
    //s_glext->glGetBufferParameteri64v(target, pname, params);
}

void glFramebufferTexture(GLenum target, GLenum attachment, GLuint texture, GLint level)
{
    s_glext->glFramebufferTexture(target, attachment, texture, level);
}

void glTexImage2DMultisample(GLenum target, GLsizei samples, GLenum internalformat, GLsizei width, GLsizei height, GLboolean fixedsamplelocations)
{
    s_glext->glTexImage2DMultisample(target, samples, internalformat, width, height, fixedsamplelocations);
}

void glTexImage3DMultisample(GLenum target, GLsizei samples, GLenum internalformat, GLsizei width, GLsizei height, GLsizei depth, GLboolean fixedsamplelocations)
{
    s_glext->glTexImage3DMultisample(target, samples, internalformat, width, height, depth, fixedsamplelocations);
}

void glGetMultisamplefv(GLenum pname, GLuint index, GLfloat *val)
{
    s_glext->glGetMultisamplefv(pname, index, val);
}

void glSampleMaski(GLuint maskNumber, GLbitfield mask)
{
    s_glext->glSampleMaski(maskNumber, mask);
}

// GL_VERSION_3_3
void glBindFragDataLocationIndexed(GLuint program, GLuint colorNumber, GLuint index, const GLchar *name)
{
    s_glext->glBindFragDataLocationIndexed(program, colorNumber, index, name);
}

GLint glGetFragDataIndex(GLuint program, const GLchar *name)
{
    return s_glext->glGetFragDataIndex(program, name);
}

void glGenSamplers(GLsizei count, GLuint *samplers)
{
    s_glext->glGenSamplers(count, samplers);
}

void glDeleteSamplers(GLsizei count, const GLuint *samplers)
{
    s_glext->glDeleteSamplers(count, samplers);
}

GLboolean glIsSampler(GLuint sampler)
{
    return s_glext->glIsSampler(sampler);
}

void glBindSampler(GLuint unit, GLuint sampler)
{
    s_glext->glBindSampler(unit, sampler);
}

void glSamplerParameteri(GLuint sampler, GLenum pname, GLint param)
{
    s_glext->glSamplerParameteri(sampler, pname, param);
}

void glSamplerParameteriv(GLuint sampler, GLenum pname, const GLint *param)
{
    s_glext->glSamplerParameteriv(sampler, pname, const_cast<GLint*>(param));
}

void glSamplerParameterf(GLuint sampler, GLenum pname, GLfloat param)
{
    s_glext->glSamplerParameterf(sampler, pname, param);
}

void glSamplerParameterfv(GLuint sampler, GLenum pname, const GLfloat *param)
{
    s_glext->glSamplerParameterfv(sampler, pname, const_cast<GLfloat*>(param));
}

void glSamplerParameterIiv(GLuint sampler, GLenum pname, const GLint *param)
{
    s_glext->glSamplerParameterIiv(sampler, pname, const_cast<GLint*>(param));
}

void glSamplerParameterIuiv(GLuint sampler, GLenum pname, const GLuint *param)
{
    s_glext->glSamplerParameterIuiv(sampler, pname, const_cast<GLuint*>(param));
}

void glGetSamplerParameteriv(GLuint sampler, GLenum pname, GLint *params)
{
    s_glext->glGetSamplerParameteriv(sampler, pname, params);
}

void glGetSamplerParameterIiv(GLuint sampler, GLenum pname, GLint *params)
{
    s_glext->glGetSamplerParameterIiv(sampler, pname, params);
}

void glGetSamplerParameterfv(GLuint sampler, GLenum pname, GLfloat *params)
{
    s_glext->glGetSamplerParameterfv(sampler, pname, params);
}

void glGetSamplerParameterIuiv(GLuint sampler, GLenum pname, GLuint *params)
{
    s_glext->glGetSamplerParameterIuiv(sampler, pname, params);
}

void glQueryCounter(GLuint id, GLenum target)
{
    s_glext->glQueryCounter(id, target);
}

void glGetQueryObjecti64v(GLuint id, GLenum pname, GLint64 *params)
{
    // TODO:
    // s_glext->glGetQueryObjecti64v(id, pname, params);
}

void glGetQueryObjectui64v(GLuint id, GLenum pname, GLuint64 *params)
{
    s_glext->glGetQueryObjectui64v(id, pname, params);
}

void glVertexAttribDivisor(GLuint index, GLuint divisor)
{
    s_glext->glVertexAttribDivisor(index, divisor);
}

void glVertexAttribP1ui(GLuint index, GLenum type, GLboolean normalized, GLuint value)
{
    // TODO:
    //s_glext->glVertexAttribP1ui(index, type, normalized, value);
}

void glVertexAttribP1uiv(GLuint index, GLenum type, GLboolean normalized, const GLuint *value)
{
    // TODO:
    // s_glext->glVertexAttribP1uiv(index, type, normalized, value);
}

void glVertexAttribP2ui(GLuint index, GLenum type, GLboolean normalized, GLuint value)
{
    // TODO:
    // s_glext->glVertexAttribP2ui(index, type, normalized, value);
}
void glVertexAttribP2uiv(GLuint index, GLenum type, GLboolean normalized, const GLuint *value)
{
    // TODO:
    //s_glext->glVertexAttribP2uiv(index, type, normalized, value);
}

void glVertexAttribP3ui(GLuint index, GLenum type, GLboolean normalized, GLuint value)
{
    // TODO:
    //s_glext->glVertexAttribP3ui(index, type, normalized, value);
}

void glVertexAttribP3uiv(GLuint index, GLenum type, GLboolean normalized, const GLuint *value)
{
    // TODO:
    //s_glext->glVertexAttribP3uiv(index, type, normalized, value);
}

void glVertexAttribP4ui(GLuint index, GLenum type, GLboolean normalized, GLuint value)
{
    // TODO:
    //s_glext->glVertexAttribP4ui9in(index, type, normalized, value);
}

void glVertexAttribP4uiv(GLuint index, GLenum type, GLboolean normalized, const GLuint *value)
{
    // TODO:
    //s_glext->glVertexAttribP4uiv(index, type, normalized, value);
}


// GL_VERSION_3_0
void glColorMaski(GLuint index, GLboolean r, GLboolean g, GLboolean b, GLboolean a)
{
    s_glext->glColorMaski(index, r, g, b, a);
}

void glGetBooleani_v(GLenum target, GLuint index, GLboolean *data)
{
    // TODO:
    //s_glext->glGetBooleani_v(target, index, data);
}

void glGetIntegeri_v(GLenum target, GLuint index, GLint *data)
{
    //s_glext->glGetIntegeri_v(target, index, data);
}

void glEnablei(GLenum target, GLuint index)
{
    s_glext->glEnablei(target, index);
}

void glDisablei(GLenum target, GLuint index)
{
    s_glext->glDisablei(target, index);
}

GLboolean glIsEnabledi(GLenum target, GLuint index)
{
    // todo:
    return GL_FALSE;
    //s_glext->glIsEnabledi(target, index);
}

void glBeginTransformFeedback(GLenum primitiveMode)
{
    s_glext->glBeginTransformFeedback(primitiveMode);
}

void glEndTransformFeedback(void)
{
    s_glext->glEndTransformFeedback();
}

void glBindBufferRange(GLenum target, GLuint index, GLuint buffer, GLintptr offset, GLsizeiptr size)
{
    s_glext->glBindBufferRange(target, index, buffer, offset, size);
}

void glBindBufferBase(GLenum target, GLuint index, GLuint buffer)
{
    s_glext->glBindBufferBase(target, index, buffer);
}

void glTransformFeedbackVaryings(GLuint program, GLsizei count, const GLchar *const*varyings, GLenum bufferMode)
{
    s_glext->glTransformFeedbackVaryings(program, count, varyings, bufferMode);
}

void glGetTransformFeedbackVarying(GLuint program, GLuint index, GLsizei bufSize, GLsizei *length, GLsizei *size, GLenum *type, GLchar *name)
{
    s_glext->glGetTransformFeedbackVarying(program, index, bufSize, length, size, type, name);
}

void glClampColor(GLenum target, GLenum clamp)
{
    s_glext->glClampColor(target, clamp);
}

void glBeginConditionalRender(GLuint id, GLenum mode)
{
    s_glext->glBeginConditionalRender(id, mode);
}

void glEndConditionalRender(void)
{
    s_glext->glEndConditionalRender();
}

void glVertexAttribIPointer(GLuint index, GLint size, GLenum type, GLsizei stride, const void *pointer)
{
    s_glext->glVertexAttribIPointer(index, size, type, stride, pointer);
}

void glGetVertexAttribIiv(GLuint index, GLenum pname, GLint *params)
{
    //s_glext->glGetVertexAttribIiv(index, pname, params);
}

void glGetVertexAttribIuiv(GLuint index, GLenum pname, GLuint *params)
{
    //s_glext->glGetVertexAttribIuiv(index, pname, params);
}

void glVertexAttribI1i(GLuint index, GLint x)
{
    //s_glext->glVertexAttribI1i(index, x);
}

void glVertexAttribI2i(GLuint index, GLint x, GLint y)
{
    //s_glext->glVertexAttribI2i(index, x, y);
}
void glVertexAttribI3i(GLuint index, GLint x, GLint y, GLint z)
{
    //s_glext->glVertexAttribI3i(index, x, y, z);
}

void glVertexAttribI4i(GLuint index, GLint x, GLint y, GLint z, GLint w)
{
    //s_glext->glVertexAttribI4i(index, x, y, z, w);
}

void glVertexAttribI1ui(GLuint index, GLuint x)
{
    //s_glext->glVertexAttribI1ui(index, x);
}

void glVertexAttribI2ui(GLuint index, GLuint x, GLuint y)
{
    //s_glext->glVertexAttribI2ui(index, x, y);
}

void glVertexAttribI3ui(GLuint index, GLuint x, GLuint y, GLuint z)
{
    //s_glext->glVertexAttribI3ui(index, x, y, z);
}

void glVertexAttribI4ui(GLuint index, GLuint x, GLuint y, GLuint z, GLuint w)
{
    //s_glext->glVertexAttribI4ui(index, x, y, z, w);
}

void glVertexAttribI1iv(GLuint index, const GLint *v)
{
    //s_glext->glVertexAttribI4ui(index, v);
}

void glVertexAttribI2iv(GLuint index, const GLint *v)
{
    //s_glext->glVertexAttribI2iv(index, v);
}

void glVertexAttribI3iv(GLuint index, const GLint *v)
{
    //s_glext->glVertexAttribI3iv(index, v);
}

void glVertexAttribI4iv(GLuint index, const GLint *v)
{
    //s_glext->glVertexAttribI4iv(index, v);
}

void glVertexAttribI1uiv(GLuint index, const GLuint *v)
{
    //s_glext->glVertexAttribI1uiv(index, v);
}

void glVertexAttribI2uiv(GLuint index, const GLuint *v)
{
    //s_glext->glVertexAttribI2uiv(index, v);
}

void glVertexAttribI3uiv(GLuint index, const GLuint *v)
{
    //s_glext->glVertexAttribI3uiv(index, v);
}

void glVertexAttribI4uiv(GLuint index, const GLuint *v)
{
    //s_glext->glVertexAttribI4uiv(index, v);
}

void glVertexAttribI4bv(GLuint index, const GLbyte *v)
{
    //s_glext->glVertexAttribI4bv(index, v);
}

void glVertexAttribI4sv(GLuint index, const GLshort *v)
{
    //s_glext->glVertexAttribI4sv(index, v);
}

void glVertexAttribI4ubv(GLuint index, const GLubyte *v)
{
    //s_glext->glVertexAttribI4ubv(index, v);
}

void glVertexAttribI4usv(GLuint index, const GLushort *v)
{
    //s_glext->glVertexAttribI4usv(index, v);
}

void glGetUniformuiv(GLuint program, GLint location, GLuint *params)
{
    s_glext->glGetUniformuiv(program, location, params);
}

void glBindFragDataLocation(GLuint program, GLuint color, const GLchar *name)
{
    s_glext->glBindFragDataLocation(program, color, name);
}

GLint glGetFragDataLocation(GLuint program, const GLchar *name)
{
    return s_glext->glGetFragDataLocation(program, name);
}

void glUniform1ui(GLint location, GLuint v0)
{
    s_glext->glUniform1ui(location, v0);
}

void glUniform2ui(GLint location, GLuint v0, GLuint v1)
{
    s_glext->glUniform2ui(location, v0, v1);
}

void glUniform3ui(GLint location, GLuint v0, GLuint v1, GLuint v2)
{
    s_glext->glUniform3ui(location, v0, v1, v2);
}

void glUniform4ui(GLint location, GLuint v0, GLuint v1, GLuint v2, GLuint v3)
{
    s_glext->glUniform4ui(location, v0, v1, v2, v3);
}

void glUniform1uiv(GLint location, GLsizei count, const GLuint *value)
{
    s_glext->glUniform1uiv(location, count, value);
}

void glUniform2uiv(GLint location, GLsizei count, const GLuint *value)
{
    s_glext->glUniform2uiv(location, count, value);
}

void glUniform3uiv(GLint location, GLsizei count, const GLuint *value)
{
    s_glext->glUniform3uiv(location, count, value);
}

void glUniform4uiv(GLint location, GLsizei count, const GLuint *value)
{
    s_glext->glUniform4uiv(location, count, value);
}

void glTexParameterIiv(GLenum target, GLenum pname, const GLint *params)
{
    s_glext->glTexParameterIiv(target, pname, params);
}

void glTexParameterIuiv(GLenum target, GLenum pname, const GLuint *params)
{
    s_glext->glTexParameterIuiv(target, pname, params);
}

void glGetTexParameterIiv(GLenum target, GLenum pname, GLint *params)
{
    //s_glext->glGetTexParameterIiv(target, pname, params);
}

void glGetTexParameterIuiv(GLenum target, GLenum pname, GLuint *params)
{
    //s_glext->glGetTexParameterIuiv(target, pname, params);
}

void glClearBufferiv(GLenum buffer, GLint drawbuffer, const GLint *value)
{
    //s_glext->glClearBufferiv(buffer, drawbuffer, value);
}

void glClearBufferuiv(GLenum buffer, GLint drawbuffer, const GLuint *value)
{
    //s_glext->glClearBufferuiv(buffer, drawbuffer, value);
}

void glClearBufferfv(GLenum buffer, GLint drawbuffer, const GLfloat *value)
{
    //s_glext->glClearBufferfv(buffer, drawbufer, value);
}

void glClearBufferfi(GLenum buffer, GLint drawbuffer, GLfloat depth, GLint stencil)
{
    //s_glext->glClearBufferfi(bufer, drawbuffer, depth, stencil);
}

const GLubyte *glGetStringi(GLenum name, GLuint index)
{
    return nullptr;
    //return s_glext->glGetStringi(name, index);
}

GLboolean glIsRenderbuffer(GLuint renderbuffer)
{
    return GL_FALSE;
    //return s_glext->glIsRenderbuffer(renderbuffer);
}

void glBindRenderbuffer(GLenum target, GLuint renderbuffer)
{
    s_glext->glBindRenderbuffer(target, renderbuffer);
}

void glDeleteRenderbuffers(GLsizei n, const GLuint *renderbuffers)
{
    s_glext->glDeleteRenderbuffers(n, renderbuffers);
}

void glGenRenderbuffers(GLsizei n, GLuint *renderbuffers)
{
    s_glext->glGenRenderbuffers(n, renderbuffers);
}

void glRenderbufferStorage(GLenum target, GLenum internalformat, GLsizei width, GLsizei height)
{
    s_glext->glRenderbufferStorage(target, internalformat, width, height);
}

void glGetRenderbufferParameteriv(GLenum target, GLenum pname, GLint *params)
{
    s_glext->glGetRenderbufferParameteriv(target, pname, params);
}

GLboolean glIsFramebuffer(GLuint framebuffer)
{
    return GL_FALSE;
    //return s_glext->glIsFramebuffer(framebuffer);
}

void glBindFramebuffer(GLenum target, GLuint framebuffer)
{
    s_glext->glBindFramebuffer(target, framebuffer);
}

void glDeleteFramebuffers(GLsizei n, const GLuint *framebuffers)
{
    s_glext->glDeleteFramebuffers(n, framebuffers);
}

void glGenFramebuffers(GLsizei n, GLuint *framebuffers)
{
    s_glext->glGenFramebuffers(n, framebuffers);
}

GLenum glCheckFramebufferStatus(GLenum target)
{
    return s_glext->glCheckFramebufferStatus(target);
}

void glFramebufferTexture1D(GLenum target, GLenum attachment, GLenum textarget, GLuint texture, GLint level)
{
    s_glext->glFramebufferTexture1D(target, attachment, textarget, texture, level);
}

void glFramebufferTexture2D(GLenum target, GLenum attachment, GLenum textarget, GLuint texture, GLint level)
{
    s_glext->glFramebufferTexture2D(target, attachment, textarget, texture, level);
}

void glFramebufferTexture3D(GLenum target, GLenum attachment, GLenum textarget, GLuint texture, GLint level, GLint zoffset)
{
    s_glext->glFramebufferTexture3D(target, attachment, textarget, texture, level, zoffset);
}

void glFramebufferRenderbuffer(GLenum target, GLenum attachment, GLenum renderbuffertarget, GLuint renderbuffer)
{
    s_glext->glFramebufferRenderbuffer(target, attachment, renderbuffertarget, renderbuffer);
}

void glGetFramebufferAttachmentParameteriv(GLenum target, GLenum attachment, GLenum pname, GLint *params)
{
    //s_glext->glGetFramebufferAttachmentParameteriv(target, attachment, pname, params);
}

void glGenerateMipmap(GLenum target)
{
    s_glext->glGenerateMipmap(target);
}

void glBlitFramebuffer(GLint srcX0, GLint srcY0, GLint srcX1, GLint srcY1, GLint dstX0, GLint dstY0, GLint dstX1, GLint dstY1, GLbitfield mask, GLenum filter)
{
    s_glext->glBlitFramebuffer(srcX0, srcY0, srcX1, srcY1, dstX0, dstY0, dstX1, dstY1, mask, filter);
}

void glRenderbufferStorageMultisample(GLenum target, GLsizei samples, GLenum internalformat, GLsizei width, GLsizei height)
{
    s_glext->glRenderbufferStorageMultisample(target, samples, internalformat, width, height);
}

void glFramebufferTextureLayer(GLenum target, GLenum attachment, GLuint texture, GLint level, GLint layer)
{
    s_glext->glFramebufferTextureLayer(target, attachment, texture, level, layer);
}

void *glMapBufferRange(GLenum target, GLintptr offset, GLsizeiptr length, GLbitfield access)
{
    return s_glext->glMapBufferRange(target, offset, length, access);
}

void glFlushMappedBufferRange(GLenum target, GLintptr offset, GLsizeiptr length)
{
    s_glext->glFlushMappedBufferRange(target, offset, length);
}

void glBindVertexArray(GLuint array)
{
    s_glext->glBindVertexArray(array);
}

void glDeleteVertexArrays(GLsizei n, const GLuint *arrays)
{
    s_glext->glDeleteVertexArrays(n, arrays);
}

void glGenVertexArrays(GLsizei n, GLuint *arrays)
{
    s_glext->glGenVertexArrays(n, arrays);
}

GLboolean glIsVertexArray(GLuint array)
{
    return s_glext->glIsVertexArray(array);
}
