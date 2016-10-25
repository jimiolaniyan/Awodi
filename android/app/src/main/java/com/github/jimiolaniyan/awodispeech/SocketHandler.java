package com.github.jimiolaniyan.awodispeech;

import android.util.Log;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.InetAddress;
import java.net.Socket;

/**
 * Created by gidimo on 29/09/2016.
 */
public class SocketHandler implements Runnable{
    private String ip;
    private int port;
    private Socket socket;
    public PrintWriter writer;
    private BufferedReader reader;
    private String tag = "LOG_TAG";

    public SocketHandler(String ip, int port){
        Log.e(tag, "Ip is " + ip + "and port is " + port);
        this.ip = ip;
        this.port = port;
    }

    public SocketHandler() {

    }

    @Override
    public void run() {
        try {
            //todo Validate IP
            Log.e(tag, "socket is " + ip + ":" + port);

            socket = new Socket("192.168.43.72", 5000);
            //reader = new BufferedReader(new InputStreamReader(socket.getInputStream()));

//            writer = new PrintWriter(new OutputStreamWriter(socket.getOutputStream()));
        } catch (Exception e){
            Log.e("AWDODI" , "Message is " + e.getMessage());
        }
    }

    public PrintWriter getWriter() {
        return writer;
    }

    public Socket getSocket() {
        return socket;
    }
}
