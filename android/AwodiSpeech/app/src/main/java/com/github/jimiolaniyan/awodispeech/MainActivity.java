package com.github.jimiolaniyan.awodispeech;

import android.content.ActivityNotFoundException;
import android.content.Intent;
import android.speech.RecognizerIntent;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.ImageButton;
import android.widget.TextView;
import android.widget.Toast;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.Socket;
import java.util.Locale;

public class MainActivity extends AppCompatActivity implements View.OnClickListener {

    private final int REQ_CODE_SPEECH_INPUT = 100;
    private TextView mSpeechText;
    private static final int PORT = 5000;
    private static final String IP = "192.168.43.72";
    private SocketHandler socketHandler;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mSpeechText = (TextView) findViewById(R.id.speech_output_text);
        ImageButton mSpeakImageButton = (ImageButton) findViewById(R.id.tapBtn);

        if(getSupportActionBar() != null) {
            getSupportActionBar().hide();
        }

        mSpeakImageButton.setOnClickListener(this);

//        socketHandler = new SocketHandler();
//        socketHandler.run();
//        socketHandler.writer.write("Howdy");
//        socketHandler.getWriter().flush();
    }

    @Override
    public void onClick(View view) {
        takeSpeechInput();
    }

    private void send(final String outputSpeech){
        Thread t = new Thread(){

            @Override
            public void run() {
                try {
                    Socket s = new Socket("192.168.137.1", 7000);
                    DataOutputStream dos = new DataOutputStream(s.getOutputStream());


                    dos.writeUTF(outputSpeech.toLowerCase());

//                    read input stream
//                    DataInputStream dis2 = new DataInputStream(s.getInputStream());
//                    InputStreamReader disR2 = new InputStreamReader(dis2);
//                    BufferedReader br = new BufferedReader(disR2);//create a BufferReader object for input

//                    Log.d("Awodi ", "read " + br.toString());
//
//                    dis2.close();
                    s.close();

                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        };
        t.start();
    }

    private void takeSpeechInput(){
        Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
        intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
        intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE, Locale.getDefault());
        intent.putExtra(RecognizerIntent.EXTRA_PROMPT, getString(R.string.speak_prompt));

        try {
            startActivityForResult(intent, REQ_CODE_SPEECH_INPUT);
        }catch (ActivityNotFoundException a){
            Toast.makeText(getApplicationContext(), getString(R.string.speak_failed), Toast.LENGTH_LONG).show();
        }
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);

        switch (requestCode){
            case REQ_CODE_SPEECH_INPUT:
                if (resultCode == RESULT_OK && data != null){
                    String receivedSpeech = data.getStringArrayListExtra(RecognizerIntent.EXTRA_RESULTS).get(0);
                    mSpeechText.setText(receivedSpeech);
                    send(receivedSpeech);
                }
                break;
        }
    }
}
