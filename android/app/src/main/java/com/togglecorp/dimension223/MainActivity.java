package com.togglecorp.dimension223;

import android.content.Intent;
import android.net.Uri;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ProgressBar;

public class MainActivity extends AppCompatActivity {

    private static final int REQUEST_CODE = 100;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        final Button loadSource = (Button) findViewById(R.id.source_button);
        assert loadSource != null;
        loadSource.setVisibility(View.INVISIBLE);

        loadSource.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent intent = new Intent(Intent.ACTION_GET_CONTENT);
                intent.setType("image/*");
                startActivityForResult(Intent.createChooser(intent, "Open folder"),REQUEST_CODE);
            }
        });

        final ProgressBar progressBar = (ProgressBar) findViewById(R.id.progress_bar);
        assert progressBar != null;
        progressBar.setVisibility(View.INVISIBLE);

        final Button loadWifi = (Button) findViewById(R.id.wifi_button);
        assert loadWifi != null;

        final EditText ipEdit = (EditText)findViewById(R.id.edit_ip);
        assert ipEdit != null;

        loadWifi.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                progressBar.setVisibility(View.VISIBLE);
                loadWifi.setVisibility(View.INVISIBLE);
                ipEdit.setVisibility(View.INVISIBLE);

                StreamLoader loader = new StreamLoader(ipEdit.getText().toString(), 1234,
                        new StreamLoader.Listener() {
                            @Override
                            public void onComplete(boolean success) {
                                progressBar.setVisibility(View.INVISIBLE);
                                loadWifi.setVisibility(View.VISIBLE);
                                ipEdit.setVisibility(View.VISIBLE);

                                if (success) {
                                    Intent intent = new Intent(MainActivity.this, VRActivity.class);
                                    startActivity(intent);
                                }
                            }
                        });
                loader.execute();
            }
        });
    }

    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if(requestCode == REQUEST_CODE){
            Uri uri = data.getData();
            Log.d("Yo", "This-> " + uri.toString());
        }
    }
}
