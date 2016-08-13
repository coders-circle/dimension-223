package com.togglecorp.dimension223;

import android.graphics.Bitmap;
import android.opengl.GLES20;
import android.opengl.GLUtils;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;

public class PointCloud {
    private float[] mVertices;
    private int[] mIndices;
    private Bitmap mTexture;

    public PointCloud(float[] vertices, int[] indices, Bitmap texture) {
        mVertices = vertices;
        mIndices = indices;
        mTexture = texture;
    }

    private FloatBuffer vertexBuffer;
    private IntBuffer indexBuffer;
    private int[] mTextureHandle = new int[1];

    public void load() {
        ByteBuffer bb = ByteBuffer.allocateDirect(mVertices.length*4);
        bb.order(ByteOrder.nativeOrder());
        vertexBuffer = bb.asFloatBuffer();
        vertexBuffer.put(mVertices);
        vertexBuffer.position(0);

        ByteBuffer ibb = ByteBuffer.allocateDirect(mIndices.length*4);
        ibb.order(ByteOrder.nativeOrder());
        indexBuffer = ibb.asIntBuffer();
        indexBuffer.put(mIndices);
        indexBuffer.position(0);

        GLES20.glGenTextures(1, mTextureHandle, 0);
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, mTextureHandle[0]);
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MIN_FILTER, GLES20.GL_NEAREST);
        GLES20.glTexParameteri(GLES20.GL_TEXTURE_2D, GLES20.GL_TEXTURE_MAG_FILTER, GLES20.GL_NEAREST);
        GLUtils.texImage2D(GLES20.GL_TEXTURE_2D, 0, mTexture, 0);
        mTexture.recycle();
    }

    public void draw(int program, float[] modelViewProjection) {
        int mvp = GLES20.glGetUniformLocation(program, "mvp");
        GLES20.glUniformMatrix4fv(mvp, 1, false, modelViewProjection, 0);

        int texture = GLES20.glGetUniformLocation(program, "uTexture");
        GLES20.glUniform1i(texture, 0);

        GLES20.glActiveTexture(GLES20.GL_TEXTURE0);
        GLES20.glBindTexture(GLES20.GL_TEXTURE_2D, mTextureHandle[0]);

        vertexBuffer.position(0);
        int position = GLES20.glGetAttribLocation(program, "position");
        GLES20.glVertexAttribPointer(position, 3, GLES20.GL_FLOAT,
                false, 4*5, vertexBuffer);
        GLES20.glEnableVertexAttribArray(position);

        vertexBuffer.position(3);
        int texcoords = GLES20.glGetAttribLocation(program, "texcoords");
        GLES20.glVertexAttribPointer(texcoords, 2, GLES20.GL_FLOAT,
                false, 4*5, vertexBuffer);
        GLES20.glEnableVertexAttribArray(texcoords);

        GLES20.glDrawElements(GLES20.GL_TRIANGLES, mIndices.length, GLES20.GL_UNSIGNED_INT, indexBuffer);
    }
}
