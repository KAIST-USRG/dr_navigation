

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.sql.Time;
import java.util.Scanner;

public class TcpIpServer {
	static private String str="";
    static boolean RUN = true;
    public static void main(String[] args) throws IOException {
    	SocketConnect sk = new SocketConnect();
    	sk.start();
    	int sibal = 0;
    	while(true) {
        	try {
				Thread.sleep(100);
	    		if(sk.getState()==Thread.State.TERMINATED) {
	    			RUN=true;
		    			sk = new SocketConnect();
		    		   sk.start();
	        	}
			} catch (Exception e) {
				// TODO Auto-generated catch block
			}
    	}

    }
    static class SocketConnect extends Thread {

    	File readFile = new File("./sensor_data.json");
    	File saveFile = new File("./state.txt");
        FileReader filereader;
        //입력 버퍼 생성
        BufferedReader bufReader;


    	ServerSocket serverSocket = null;
        Socket socket = null;

        OutputStream outputStream = null;
        DataOutputStream dataOutputStream = null;

        InputStream inputStream = null;
        DataInputStream dataInputStream = null;
        Object lock = new Object();

        public void run() {
        	try {
							if(serverSocket==null){
	                serverSocket = new ServerSocket(9000);
							}
                System.out.println("클라이언트로부터 데이터 전송받을 준비 완료");

                socket = serverSocket.accept();
                System.out.println("클라이언트 연결 완료");
                System.out.println("socket : " + socket);

                inputStream = socket.getInputStream();
                dataInputStream = new DataInputStream(inputStream);

                outputStream = socket.getOutputStream();
                dataOutputStream = new DataOutputStream(outputStream);
    			int count=0;
                while (RUN) {
            		filereader = new FileReader(readFile);
        			bufReader = new BufferedReader(filereader);
        			str = bufReader.readLine();
                	Thread.sleep(500);
//                	System.out.println("count"+count);
                	if(dataInputStream.available()>0) {
                		count=0;
                		String clientMessage = dataInputStream.readUTF();
//                        System.out.println(clientMessage);
                		FileOutputStream fos = new FileOutputStream(saveFile);
                        fos.write(clientMessage.getBytes());

                        fos.close(); // 파일을 닫는다.
                   		dataOutputStream.writeUTF(str);
                   		dataOutputStream.flush();
                	} else {
                		count++;
                	}
                	if(count>5) {
                		close();
                		break;
                	}
                }

            } catch (Exception e) {
               // e.printStackTrace();
            } finally {
            		close();
            }
        }

    	void close() {
        	RUN=false;
    		try {
                if (dataOutputStream != null) dataOutputStream.close();
                if (outputStream != null) outputStream.close();
                if (dataInputStream != null) dataInputStream.close();
                if (inputStream != null) inputStream.close();
    		}
    		catch (Exception e) {
                e.printStackTrace();
            }
    	}
    }
}
