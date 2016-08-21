package com.togglecorp.dimension223;

import android.content.Context;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.AsyncTask;
import android.util.Log;

import java.io.DataInputStream;
import java.io.IOException;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class StreamLoader extends AsyncTask<Void, Void, Void> {

    public interface Listener {
        void onComplete(boolean success);
    }

    private final String mIp;
    private final int mPort;
    private final Listener mListener;

    private boolean mSuccess;

    public StreamLoader(String ip, int port, Listener listener) {
        mIp = ip;
        mPort = port;
        mListener = listener;
    }

    @Override
    protected Void doInBackground(Void... voids) {
        try {
            Socket socket = new Socket(mIp, mPort);
            DataInputStream inputStream = new DataInputStream(socket.getInputStream());

            byte[] buf_size = new byte[4];
            byte[] bytes = new byte[8192];
            int len, temp;

            inputStream.read(buf_size);
            int numPointClouds = ByteBuffer.wrap(buf_size).order(ByteOrder.nativeOrder()).getInt();

            for (int i=0; i<numPointClouds; ++i) {
                inputStream.read(buf_size);
                int numVertices = ByteBuffer.wrap(buf_size).order(ByteOrder.nativeOrder()).getInt();

                float[] vertices = new float[numVertices];
                temp=0;
                while ((len = inputStream.read(bytes, 0, Math.min(8192, (numVertices-temp)*4)))>0) {
                    ByteBuffer vBuffer = ByteBuffer.wrap(bytes).order(ByteOrder.nativeOrder());
                    for (int j = 0; j < len/4; ++j) {
                        vertices[temp] = vBuffer.getFloat();
                        temp++;
                    }
                }
                Log.d("Loading Point Cloud", "Got "+numVertices + " vertices.");

                inputStream.read(buf_size);
                int numIndices = ByteBuffer.wrap(buf_size).order(ByteOrder.nativeOrder()).getInt();
                int[] indices = new int[numIndices];
                temp=0;
                while ((len = inputStream.read(bytes, 0, Math.min(8192, (numIndices-temp)*4)))>0) {
                    ByteBuffer iBuffer = ByteBuffer.wrap(bytes).order(ByteOrder.nativeOrder());
                    for (int j = 0; j < len/4; ++j) {
                        indices[temp] = iBuffer.getInt();
                        temp++;
                    }
                }
                Log.d("Loading Point Cloud", "Got "+numIndices + " indices.");

                inputStream.read(buf_size);
                int textureSize = ByteBuffer.wrap(buf_size).order(ByteOrder.nativeOrder()).getInt();
                byte[] texture = new byte[textureSize];
                temp=0;
                while ((len = inputStream.read(bytes, 0, Math.min(8192, (textureSize-temp))))>0) {
                    ByteBuffer tBuffer = ByteBuffer.wrap(bytes).order(ByteOrder.nativeOrder());
                    tBuffer.get(texture, temp, len);
                    temp += len;
                }
                Log.d("Loading Point Cloud", "Got "+textureSize + " bytes of texture.");

                Bitmap bitmap = BitmapFactory.decodeByteArray(texture, 0, textureSize);

                PointCloud pc = new PointCloud(vertices, indices, bitmap);
                VRActivity.pointsClouds.add(pc);
            }

            socket.close();
            mSuccess = true;
        } catch (IOException e) {
            e.printStackTrace();
            mSuccess = false;
        }

        return null;
    }

    @Override
    protected void onPostExecute(Void result) {
        mListener.onComplete(mSuccess);
    }
}
