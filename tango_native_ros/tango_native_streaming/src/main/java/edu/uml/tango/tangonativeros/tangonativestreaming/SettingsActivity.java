package edu.uml.tango.tangonativeros.tangonativestreaming;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Spinner;
import android.widget.TextView;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStreamReader;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.util.Collections;
import java.util.List;

//TODO: move file code to class?

public class SettingsActivity extends Activity {

    public static String ros_master = "http://10.0.4.14:11311",
                         ros_ip = "",
                         tango_prefix = "tango_brain_0/",
                         namespace = "tango_brain_0";
    public TextView err_no_master;

    ToggleUI masterIPComponent;
    ToggleUI rosIPComponent;
    ToggleUI rosPrefixComponent;
    ToggleUI namespaceCompentent;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        Intent intent = getIntent();
        if (intent.hasExtra("ROS_MASTER"))
            ros_master = intent.getStringExtra("ROS_MASTER");
        if (intent.hasExtra("ROS_IP"))
            ros_ip = intent.getStringExtra("ROS_IP");
        if (intent.hasExtra("TANGO_PREFIX"))
            tango_prefix = intent.getStringExtra("TANGO_PREFIX");
        if (intent.hasExtra("NAMESPACE"))
            namespace = intent.getStringExtra("NAMESPACE");

        masterIPComponent = new ToggleUI();
        rosIPComponent = new ToggleUI();
        rosPrefixComponent = new ToggleUI();
        namespaceCompentent = new ToggleUI();

        ros_ip = getIPAddress(true);
        setContentView(R.layout.activity_settings);

        masterIPComponent.spinner = (Spinner) findViewById(R.id.MASTER_IP_SPINNER);
        rosIPComponent.spinner = (Spinner) findViewById(R.id.ROS_NODE_IP_SPINNER);
        rosPrefixComponent.spinner = (Spinner) findViewById(R.id.ROS_PREFIX_SPINNER);
        namespaceCompentent.spinner = (Spinner) findViewById(R.id.NAMESPACE_SPINNER);

        masterIPComponent.editTxt = (EditText) findViewById(R.id.MASTER_IP_EDIT);
        rosPrefixComponent.editTxt = (EditText) findViewById(R.id.ROS_PREFIX_EDIT);
        rosIPComponent.editTxt = (EditText) findViewById(R.id.ROS_NODE_IP_EDIT);
        namespaceCompentent.editTxt = (EditText) findViewById(R.id.NAMESPACE_EDIT);

        masterIPComponent.toggleBtn = (Button) findViewById(R.id.TOGGLE_MASTER_IP_BTN);
        rosPrefixComponent.toggleBtn = (Button) findViewById(R.id.TOGGLE_ROS_PREFIX_BTN);
        rosIPComponent.toggleBtn = (Button) findViewById(R.id.TOGGLE_ROS_NODE_IP_BTN);
        namespaceCompentent.toggleBtn = (Button) findViewById(R.id.TOGGLE_NAMESPACE_BTN);

        err_no_master = (TextView) findViewById(R.id.ERR_NO_MASTER_LBL);

        if (savedInstanceState != null) {
            ros_master = savedInstanceState.getString("ROS_MASTER");
            ros_ip = savedInstanceState.getString("ROS_IP");
            tango_prefix = savedInstanceState.getString("TANGO_PREFIX");
            namespace = savedInstanceState.getString("NAMESPACE");
        }

        if (!ros_ip.equals("")) {
            rosIPComponent.editTxt.setText(ros_ip);
        } else {
            rosIPComponent.editTxt.setText(R.string.tangoAddrError);
        }

        masterIPComponent.fileName = "previousDataMasterIP";
        rosIPComponent.fileName = "previousDataRosIP";
        rosPrefixComponent.fileName = "previousDataRosPrefix";
        namespaceCompentent.fileName = "previousDataNamespace";

        masterIPComponent.newDataButtonString = "Enter New IP";
        rosIPComponent.newDataButtonString = "Enter New IP";
        rosPrefixComponent.newDataButtonString = "Enter New Prefix";
        namespaceCompentent.newDataButtonString = "Enter New Namespace";

        masterIPComponent.dataStr.clear();
        rosIPComponent.dataStr.clear();
        rosPrefixComponent.dataStr.clear();
        namespaceCompentent.dataStr.clear();

        readFile(masterIPComponent.fileName, masterIPComponent.dataStr);
        readFile(rosIPComponent.fileName, rosIPComponent.dataStr);
        readFile(rosPrefixComponent.fileName, rosPrefixComponent.dataStr);
        readFile(namespaceCompentent.fileName, namespaceCompentent.dataStr);

        if(rosPrefixComponent.dataStr.size() == 0) {
            rosPrefixComponent.editTxt.setText(R.string.defaultPrefix);
        }
        if(namespaceCompentent.dataStr.size() == 0) {
            namespaceCompentent.editTxt.setText(R.string.defaultNamespace);
        }

        masterIPComponent.initSpinner(this);
        rosIPComponent.initSpinner(this);
        rosPrefixComponent.initSpinner(this);
        namespaceCompentent.initSpinner(this);
    }

    @Override
    protected void onResume() {
        super.onResume();
    }

    public static String getIPAddress(boolean useIPv4) {
        try {
            List<NetworkInterface> interfaces = Collections.list(NetworkInterface.getNetworkInterfaces());
            for (NetworkInterface intf : interfaces) {
                List<InetAddress> addrs = Collections.list(intf.getInetAddresses());
                for (InetAddress addr : addrs) {
                    if (!addr.isLoopbackAddress()) {
                        String sAddr = addr.getHostAddress();
                        //boolean isIPv4 = InetAddressUtils.isIPv4Address(sAddr);
                        boolean isIPv4 = sAddr.indexOf(':')<0;

                        if (useIPv4) {
                            if (isIPv4)
                                return sAddr;
                        } else {
                            if (!isIPv4) {
                                int delim = sAddr.indexOf('%'); // drop ip6 zone suffix
                                return delim<0 ? sAddr.toUpperCase() : sAddr.substring(0, delim).toUpperCase();
                            }
                        }
                    }
                }
            }
        } catch (Exception ex) {
            Log.e("Get Tango IP Error","");
        } // for now eat exceptions
        return "";
    }
    public void startStreaming(View view) {

        Intent intent = new Intent(this, NativeStreamingActivity.class);

        ros_master = masterIPComponent.dataFromUser();
        if(masterIPComponent.isNew) {
            writeFile(masterIPComponent.fileName, masterIPComponent.dataStr, ros_master);
        }
        else{
            updateOrder(masterIPComponent.fileName,masterIPComponent.dataStr, ros_master);
        }
        ros_ip = rosIPComponent.dataFromUser();
        if(rosIPComponent.isNew) {
            writeFile(rosIPComponent.fileName, rosIPComponent.dataStr, ros_ip);
        }
        else{
            updateOrder(rosIPComponent.fileName,rosIPComponent.dataStr, ros_ip);
        }
        tango_prefix = rosPrefixComponent.dataFromUser();
        if(rosPrefixComponent.isNew) {
            writeFile(rosPrefixComponent.fileName, rosPrefixComponent.dataStr, tango_prefix);
        }
        else{
            updateOrder(rosPrefixComponent.fileName,rosPrefixComponent.dataStr, tango_prefix);
        }
        namespace = namespaceCompentent.dataFromUser();
        if(namespaceCompentent.isNew) {
            writeFile(namespaceCompentent.fileName, namespaceCompentent.dataStr, namespace);
        }
        else{
            updateOrder(namespaceCompentent.fileName,namespaceCompentent.dataStr, namespace);
        }

        Log.d("ROS Master URI: ", ros_master);
        Log.d("Tango IP: ", ros_ip);
        Log.d("Tango prefix: ", tango_prefix);
        Log.d("Tango Namespace: ", namespace);

        intent.putExtra("ROS_MASTER", ros_master);
        intent.putExtra("ROS_IP", ros_ip);
        intent.putExtra("TANGO_PREFIX", tango_prefix);
        intent.putExtra("NAMESPACE", namespace);
        intent.addFlags(Intent.FLAG_ACTIVITY_NO_HISTORY);
        intent.addFlags(Intent.FLAG_ACTIVITY_RESET_TASK_IF_NEEDED);
        intent.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK | Intent.FLAG_ACTIVITY_CLEAR_TASK);
        startActivity(intent);
        finish();
    }

    @Override
    public void onSaveInstanceState(Bundle savedInstanceState) {
        super.onSaveInstanceState(savedInstanceState);
        ros_master = masterIPComponent.editTxt.getText().toString();
        ros_ip = rosIPComponent.editTxt.getText().toString();
        tango_prefix = rosPrefixComponent.editTxt.getText().toString();
        namespace = namespaceCompentent.editTxt.getText().toString();
        savedInstanceState.putString("ROS_MASTER", ros_master);
        savedInstanceState.putString("ROS_IP", ros_ip);
        savedInstanceState.putString("TANGO_PREFIX", tango_prefix);
        savedInstanceState.putString("NAMESPACE", namespace);
    }

    @Override
    public void onRestoreInstanceState(Bundle savedInstanceState) {
        super.onRestoreInstanceState(savedInstanceState);
        ros_master = savedInstanceState.getString("ROS_MASTER");
        ros_ip = savedInstanceState.getString("ROS_IP");
        tango_prefix = savedInstanceState.getString("TANGO_PREFIX");
        namespace = savedInstanceState.getString("NAMESPACE");
        masterIPComponent.editTxt = (EditText) findViewById(R.id.MASTER_IP_EDIT);
        rosIPComponent.editTxt = (EditText) findViewById(R.id.ROS_NODE_IP_EDIT);
        rosPrefixComponent.editTxt = (EditText) findViewById(R.id.ROS_PREFIX_EDIT);
        namespaceCompentent.editTxt = (EditText) findViewById(R.id.NAMESPACE_EDIT);
        masterIPComponent.editTxt.setText(ros_master);
        rosIPComponent.editTxt.setText(ros_ip);
        rosPrefixComponent.editTxt.setText(tango_prefix);
        namespaceCompentent.editTxt.setText(namespace);
    }

    public void toggleMasterIP(View view) {
        masterIPComponent.toggleBtns();
    }

    public void toggleRosNodeIP(View view) {
        rosIPComponent.toggleBtns();
    }

    public void toggleRosPrefix(View view) {
        rosPrefixComponent.toggleBtns();
    }

    public void toggleNewNamespace(View view) {
        namespaceCompentent.toggleBtns();
    }

    public void readFile(String fileName, List<String> str){
        String line;
        try {
            FileInputStream fis = openFileInput(fileName);
            BufferedReader in = new BufferedReader(new InputStreamReader(fis));
            line = in.readLine();
            while (line != null) {
                str.add(line);
                line = in.readLine();
            }
            in.close();
            fis.close();
        }
        catch (Throwable t) {
            Log.e("Read File Error","");
        }
    }

    public void writeFile(String fileName, List<String> str, String newString){
        int index;

            //Removes the oldest ip to make space for the new one
            while (str.size() >= 5) {
                str.remove(0);
            }
            str.add(newString);

            try {
                FileOutputStream fos = openFileOutput(fileName, Context.MODE_PRIVATE);

                for(index = 0; index < str.size(); index ++) {
                    fos.write(str.get(index).getBytes());
                    fos.write(System.getProperty("line.separator").getBytes());
                }
                fos.close();
            }
            catch (Throwable t) {
                Log.e("Write File Error","");
            }
        }

    public void updateOrder(String fileName, List<String> str, String newString) {
        int index;

        for(index = 0; index < str.size(); index++) {
            if (str.get(index).equals(newString)) {
                str.remove(index);
                break;
            }
        }
            str.add(newString);

            try {
                FileOutputStream fos = openFileOutput(fileName, Context.MODE_PRIVATE);

                for(index = 0; index < str.size(); index ++) {
                    fos.write(str.get(index).getBytes());
                    fos.write(System.getProperty("line.separator").getBytes());
                }
                fos.close();
            }
            catch (Throwable t) {
                Log.e("Update Order File Error","");
            }
    }
}