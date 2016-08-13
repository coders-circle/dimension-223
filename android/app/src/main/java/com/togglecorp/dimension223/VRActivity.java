package com.togglecorp.dimension223;

import android.opengl.GLES20;
import android.opengl.Matrix;
import android.os.Bundle;

import com.google.vr.sdk.base.Eye;
import com.google.vr.sdk.base.GvrActivity;
import com.google.vr.sdk.base.GvrView;
import com.google.vr.sdk.base.HeadTransform;
import com.google.vr.sdk.base.Viewport;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;

import javax.microedition.khronos.egl.EGLConfig;

public class VRActivity extends GvrActivity implements GvrView.StereoRenderer {

    public static List<PointCloud> pointsClouds = new ArrayList<>();

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_vr);
        GvrView gvrView = (GvrView) findViewById(R.id.gvr_view);
        gvrView.setRenderer(this);
        setGvrView(gvrView);
    }

    private float[] mHeadView = new float[16];
    private float[] mView = new float[16];
    private float[] mCamera = new float[16];
    private float[] mModelView = new float[16];
    private float[] mModelViewProjection = new float[16];
    private float[] mModelCube = new float[16];

    private int mProgram;

    @Override
    public void onNewFrame(HeadTransform headTransform) {
        headTransform.getHeadView(mHeadView, 0);

        // Build the Model part of the ModelView matrix.
//        Matrix.rotateM(mModelCube, 0, 0.3f, 0, 0, 1);

        // Build the camera matrix and apply it to the ModelView.
        Matrix.setLookAtM(mCamera, 0, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f);
    }

    @Override
    public void onDrawEye(Eye eye) {
        GLES20.glClearColor(0.8f, 0.8f, 0.8f, 1.0f);
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT | GLES20.GL_DEPTH_BUFFER_BIT);

        // Apply the eye transformation to the camera.
        Matrix.multiplyMM(mView, 0, eye.getEyeView(), 0, mCamera, 0);

        // Build the ModelView and ModelViewProjection matrices
        // for calculating cube position and light.
        float[] perspective = eye.getPerspective(0.1f, 1000.0f);
        Matrix.multiplyMM(mModelView, 0, mView, 0, mModelCube, 0);
        Matrix.multiplyMM(mModelViewProjection, 0, perspective, 0, mModelView, 0);

        GLES20.glUseProgram(mProgram);

        for (PointCloud pc: pointsClouds)
            pc.draw(mProgram, mModelViewProjection);
    }

    @Override
    public void onFinishFrame(Viewport viewport) {

    }

    @Override
    public void onSurfaceChanged(int i, int i1) {

    }

    @Override
    public void onSurfaceCreated(EGLConfig eglConfig) {
        GLES20.glEnable(GLES20.GL_DEPTH_TEST);

        Matrix.setIdentityM(mModelCube, 0);

        int vertexShader = Utilities.loadGLShader(this, GLES20.GL_VERTEX_SHADER, R.raw.simple_vs);
        int fragmentShader = Utilities.loadGLShader(this, GLES20.GL_FRAGMENT_SHADER, R.raw.simple_fs);

        mProgram = GLES20.glCreateProgram();
        GLES20.glAttachShader(mProgram, vertexShader);
        GLES20.glAttachShader(mProgram, fragmentShader);
        GLES20.glLinkProgram(mProgram);

        for (PointCloud pc: pointsClouds)
            pc.load();
    }

    @Override
    public void onRendererShutdown() {

    }
}
