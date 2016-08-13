package com.togglecorp.dimension223;

import android.content.Intent;
import android.net.Uri;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;

public class MainActivity extends AppCompatActivity {

    private static final int REQUEST_CODE = 100;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        Button loadSource = (Button) findViewById(R.id.source_button);
        if (loadSource != null)
            loadSource.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
    //                Intent intent = new Intent(Intent.ACTION_GET_CONTENT);
    //                intent.setType("image/*");
    //                startActivityForResult(Intent.createChooser(intent, "Open folder"),REQUEST_CODE);
                    Intent vrIntent = new Intent(MainActivity.this, VRActivity.class);
                    startActivity(vrIntent);
                }
            });



        Button loadWifi = (Button) findViewById(R.id.wifi_button);
        if (loadWifi != null) {
            loadWifi.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View view) {
                    StreamLoader loader = new StreamLoader(MainActivity.this, "192.168.100.12", 1234);
                    loader.execute();
                }
            });
        }
    }

    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if(requestCode == REQUEST_CODE){
            Uri uri = data.getData();
            Log.d("Yo", "This-> " + uri.toString());
        }
    }
}
