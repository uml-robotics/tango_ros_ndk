package edu.uml.tango.tangonativeros.tangonativestreaming;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.ArrayAdapter;
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
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

//TODO: if size is 0 then use the defaults for tango_prefix ros_ip and namespace
    //tango_brain_0/
    //tango_brain_0
//TODO: Make all spinner/file code cleaner
    //make a spinner class

public class SettingsActivity extends Activity {
    public static String ros_master = "",
                         ros_ip = "",
                         tango_prefix = "",
                         namespace = "";
    public TextView ros_master_edit, tango_addr_edit, prefix_edit, tango_namespace_edit, err_no_master;
    /* private boolean isNewMasterIP = false;
    private Spinner masterIPSpinner;
    private EditText enterNewMasterIPEdit;
    private Button enterNewMasterIPBtn;

    private boolean isNewRosNodeIP = false;
    private Spinner rosNodeIPSpinner;
    private EditText enterNewRosNodeIPEdit;
    private Button enterNewRosNodeIPBtn;

    private boolean isNewRosPrefix = false;
    private Spinner rosPrefixSpinner;
    private EditText enterNewRosPrefixEdit;
    private Button enterNewRosPrefixBtn;

    private boolean isNewNamespace = false;
    private Spinner namespaceSpinner;
    private EditText enterNewNamespaceEdit;
    private Button enterNewNamespaceBtn;

    List<String> masterPrefixDataStr;
    List<String> masterIPDataStr;
    List<String> portDataStr;
    List<String> rosIPDataStr;
    List<String> rosPrefixDataStr;
    List<String> namespaceDataStr;*/

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
        ros_master_edit = (TextView) findViewById(R.id.MASTER_IP_EDIT);

        tango_addr_edit = (TextView) findViewById(R.id.ROS_NODE_IP_EDIT);
        prefix_edit = (TextView) findViewById(R.id.ROS_PREFIX_EDIT);
        tango_namespace_edit = (TextView) findViewById(R.id.NAMESPACE_EDIT);

        err_no_master = (TextView) findViewById(R.id.ERR_NO_MASTER_LBL);

        if (savedInstanceState != null) {
            ros_master = savedInstanceState.getString("ROS_MASTER");
            ros_ip = savedInstanceState.getString("ROS_IP");
            tango_prefix = savedInstanceState.getString("TANGO_PREFIX");
            namespace = savedInstanceState.getString("NAMESPACE");
        }
        ros_master_edit.setText(ros_master);

        if (!ros_ip.equals("")) {
            tango_addr_edit.setText(ros_ip);
        } else {
            tango_addr_edit.setText("tango_addr_error");
        }

        prefix_edit.setText(tango_prefix);

        tango_namespace_edit.setText(namespace);
        /*
        masterPrefixDataStr = readFile("previousDataMasterPrefix");
        masterIPDataStr = readFile("previousDataMasterIP");
        portDataStr = readFile("previousDataPort");
        rosIPDataStr = readFile("previousDataRosIP");
        rosPrefixDataStr = readFile("previousDataRosPrefix");
        namespaceDataStr = readFile("previousDataNamespace");

        masterIPSpinner = (Spinner) findViewById(R.id.MASTER_IP_SPINNER);
        ArrayAdapter<String> adapterMasterIP = new ArrayAdapter<String>(this, android.R.layout.simple_spinner_item, reverseStr(masterIPDataStr));
        adapterMasterIP.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        masterIPSpinner.setAdapter(adapterMasterIP);

        rosNodeIPSpinner = (Spinner) findViewById(R.id.ROS_NODE_IP_SPINNER);
        ArrayAdapter<String> adapterRosNodeIP = new ArrayAdapter<String>(this, android.R.layout.simple_spinner_item, reverseStr(rosIPDataStr));
        adapterRosNodeIP.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        rosNodeIPSpinner.setAdapter(adapterRosNodeIP);

        rosPrefixSpinner = (Spinner) findViewById(R.id.ROS_PREFIX_SPINNER);
        ArrayAdapter<String> adapterRosPrefix = new ArrayAdapter<String>(this, android.R.layout.simple_spinner_item, reverseStr(rosPrefixDataStr));
        adapterRosPrefix.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        rosPrefixSpinner.setAdapter(adapterRosPrefix);

        namespaceSpinner = (Spinner) findViewById(R.id.NAMESPACE_SPINNER);
        ArrayAdapter<String> adapterNamespace = new ArrayAdapter<String>(this, android.R.layout.simple_spinner_item, reverseStr(namespaceDataStr));
        adapterNamespace.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        namespaceSpinner.setAdapter(adapterNamespace);*/

        //TODO: Move this to the class if possible?
       /* masterIPComponent.spinner = (Spinner) findViewById(R.id.MASTER_IP_SPINNER);
        rosPrefixComponent.spinner = (Spinner) findViewById(R.id.ROS_PREFIX_SPINNER);
        rosIPComponent.spinner = (Spinner) findViewById(R.id.ROS_NODE_IP_SPINNER);
        namespaceCompentent.spinner = (Spinner) findViewById(R.id.NAMESPACE_SPINNER);

        masterIPComponent.editTxt = (EditText) findViewById(R.id.MASTER_IP_EDIT);
        rosPrefixComponent.editTxt = (EditText) findViewById(R.id.ROS_PREFIX_EDIT);
        rosIPComponent.editTxt = (EditText) findViewById(R.id.ROS_NODE_IP_EDIT);
        namespaceCompentent.editTxt = (EditText) findViewById(R.id.NAMESPACE_EDIT);

        masterIPComponent.toggleBtn = (Button) findViewById(R.id.TOGGLE_MASTER_IP_BTN);
        rosPrefixComponent.toggleBtn = (Button) findViewById(R.id.TOGGLE_ROS_PREFIX_BTN);
        rosIPComponent.toggleBtn = (Button) findViewById(R.id.TOGGLE_ROS_NODE_IP_BTN);
        namespaceCompentent.toggleBtn = (Button) findViewById(R.id.TOGGLE_NAMESPACE_BTN);



        masterIPComponent.initSpinner(this);
        rosIPComponent.initSpinner(this);
        rosPrefixComponent.initSpinner(this);
        namespaceCompentent.initSpinner(this);*/

        masterIPComponent.fileName = "previousDataMasterIP";
        rosIPComponent.fileName = "previousDataRosIP";
        rosPrefixComponent.fileName = "previousDataRosPrefix";
        namespaceCompentent.fileName = "previousDataNamespace";

        masterIPComponent.initData();
        rosIPComponent.initData();
        rosPrefixComponent.initData();
        namespaceCompentent.initData();
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
        } catch (Exception ex) { } // for now eat exceptions
        return "";
    }
    public void startStreaming(View view) {

        Intent intent = new Intent(this, NativeStreamingActivity.class);

        masterIPComponent.spinner = (Spinner) findViewById(R.id.MASTER_IP_SPINNER);
        rosPrefixComponent.spinner = (Spinner) findViewById(R.id.ROS_PREFIX_SPINNER);
        rosIPComponent.spinner = (Spinner) findViewById(R.id.ROS_NODE_IP_SPINNER);
        namespaceCompentent.spinner = (Spinner) findViewById(R.id.NAMESPACE_SPINNER);

        masterIPComponent.editTxt = (EditText) findViewById(R.id.MASTER_IP_EDIT);
        rosPrefixComponent.editTxt = (EditText) findViewById(R.id.ROS_PREFIX_EDIT);
        rosIPComponent.editTxt = (EditText) findViewById(R.id.ROS_NODE_IP_EDIT);
        namespaceCompentent.editTxt = (EditText) findViewById(R.id.NAMESPACE_EDIT);

        masterIPComponent.toggleBtn = (Button) findViewById(R.id.TOGGLE_MASTER_IP_BTN);
        rosPrefixComponent.toggleBtn = (Button) findViewById(R.id.TOGGLE_ROS_PREFIX_BTN);
        rosIPComponent.toggleBtn = (Button) findViewById(R.id.TOGGLE_ROS_NODE_IP_BTN);
        namespaceCompentent.toggleBtn = (Button) findViewById(R.id.TOGGLE_NAMESPACE_BTN);

        ros_master = masterIPComponent.dataFromUser();
        ros_ip = rosIPComponent.dataFromUser();
        tango_prefix = rosPrefixComponent.dataFromUser();
        namespace = namespaceCompentent.dataFromUser();
        /*
        if(!isNewMasterIP) {
            masterIPSpinner = (Spinner) findViewById(R.id.MASTER_IP_SPINNER);
            ros_master = masterIPSpinner.getSelectedItem().toString();
        }
        else {
            enterNewMasterIPEdit = (EditText) findViewById(R.id.MASTER_IP_EDIT);
            ros_master = enterNewMasterIPEdit.getText().toString();
            writeFile("previousDataMasterIP", masterIPDataStr, ros_master);
        }

        if(!isNewRosNodeIP) {
            rosNodeIPSpinner = (Spinner) findViewById(R.id.ROS_NODE_IP_SPINNER);
            ros_ip = rosNodeIPSpinner.getSelectedItem().toString();
        }
        else {
            enterNewRosNodeIPEdit = (EditText) findViewById(R.id.ROS_NODE_IP_EDIT);
            ros_ip = enterNewRosNodeIPEdit.getText().toString();
            writeFile("previousDataRosIP", rosIPDataStr, ros_ip);
        }

        if(!isNewRosPrefix) {
            rosPrefixSpinner = (Spinner) findViewById(R.id.ROS_PREFIX_SPINNER);
            tango_prefix = rosPrefixSpinner.getSelectedItem().toString();
        }
        else {
            enterNewRosPrefixEdit = (EditText) findViewById(R.id.ROS_PREFIX_EDIT);
            tango_prefix = enterNewRosPrefixEdit.getText().toString();
           writeFile("previousDataRosPrefix", rosPrefixDataStr, tango_prefix);
        }

        if(!isNewNamespace) {
            namespaceSpinner = (Spinner) findViewById(R.id.NAMESPACE_SPINNER);
            namespace = namespaceSpinner.getSelectedItem().toString();
        }
        else {
            enterNewNamespaceEdit = (EditText) findViewById(R.id.NAMESPACE_EDIT);
            namespace = enterNewNamespaceEdit.getText().toString();
            writeFile("previousDataNamespace", namespaceDataStr, namespace);
        }*/

        Log.d("ROS Master URI: ", ros_master);
        Log.d("Tango IP: ", ros_ip);
        Log.d("Tango prefix: ", tango_prefix);
        Log.d("Tango Namespace: ", namespace);

        intent.putExtra("ROS_MASTER", ros_master);
        intent.putExtra("ROS_IP", ros_ip);
        intent.putExtra("TANGO_PREFIX", tango_prefix);
        intent.putExtra("NAMESPACE", namespace);
        startActivity(intent);
        finish();
    }

    @Override
    public void onSaveInstanceState(Bundle savedInstanceState) {
        super.onSaveInstanceState(savedInstanceState);
        ros_master = ros_master_edit.getText().toString();
        ros_ip = tango_addr_edit.getText().toString();
        tango_prefix = prefix_edit.getText().toString();
        namespace = tango_namespace_edit.getText().toString();
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
        ros_master_edit = (TextView) findViewById(R.id.MASTER_IP_EDIT);
        tango_addr_edit = (TextView) findViewById(R.id.ROS_NODE_IP_EDIT);
        prefix_edit = (TextView) findViewById(R.id.ROS_PREFIX_EDIT);
        tango_namespace_edit = (TextView) findViewById(R.id.NAMESPACE_EDIT);
        ros_master_edit.setText(ros_master);
        tango_addr_edit.setText(ros_ip);
        prefix_edit.setText(tango_prefix);
        tango_namespace_edit.setText(namespace);
    }

    public void toggleMasterIP(View view) {
        masterIPComponent.spinner = (Spinner) findViewById(R.id.MASTER_IP_SPINNER);
        masterIPComponent.editTxt = (EditText) findViewById(R.id.MASTER_IP_EDIT);
        masterIPComponent.toggleBtn = (Button) findViewById(R.id.TOGGLE_MASTER_IP_BTN);
        masterIPComponent.toggleBtns();
    }

    public void toggleRosNodeIP(View view) {
        rosIPComponent.spinner = (Spinner) findViewById(R.id.ROS_NODE_IP_SPINNER);
        rosIPComponent.editTxt = (EditText) findViewById(R.id.ROS_NODE_IP_EDIT);
        rosIPComponent.toggleBtn = (Button) findViewById(R.id.TOGGLE_ROS_NODE_IP_BTN);
        rosIPComponent.toggleBtns();
    }

    public void toggleRosPrefix(View view) {
        rosPrefixComponent.spinner = (Spinner) findViewById(R.id.ROS_PREFIX_SPINNER);
        rosPrefixComponent.editTxt = (EditText) findViewById(R.id.ROS_PREFIX_EDIT);
        rosPrefixComponent.toggleBtn = (Button) findViewById(R.id.TOGGLE_ROS_PREFIX_BTN);
        rosPrefixComponent.toggleBtns();
    }

    public void toggleNewNamespace(View view) {
        namespaceCompentent.spinner = (Spinner) findViewById(R.id.NAMESPACE_SPINNER);
        namespaceCompentent.editTxt = (EditText) findViewById(R.id.NAMESPACE_EDIT);
        namespaceCompentent.toggleBtn = (Button) findViewById(R.id.TOGGLE_NAMESPACE_BTN);
        namespaceCompentent.toggleBtns();
    }

/*
    public void toggleMasterIP(View view) {
        masterIPSpinner = (Spinner) findViewById(R.id.MASTER_IP_SPINNER);
        enterNewMasterIPBtn = (Button) findViewById(R.id.TOGGLE_MASTER_IP_BTN);
        enterNewMasterIPEdit = (EditText) findViewById(R.id.MASTER_IP_EDIT);
        if(!isNewMasterIP) {
            isNewMasterIP = true;
            masterIPSpinner.setVisibility(View.GONE);
            enterNewMasterIPBtn.setText("Back to list");
            enterNewMasterIPEdit.setVisibility(View.VISIBLE);
        }
        else{
            isNewMasterIP = false;
            masterIPSpinner.setVisibility(View.VISIBLE);
            enterNewMasterIPBtn.setText("Set New IP");
            enterNewMasterIPEdit.setVisibility(View.GONE);
        }
    }

    public void toggleRosNodeIP(View view) {
        rosNodeIPSpinner = (Spinner) findViewById(R.id.ROS_NODE_IP_SPINNER);
        enterNewRosNodeIPBtn = (Button) findViewById(R.id.TOGGLE_ROS_NODE_IP_BTN);
        enterNewRosNodeIPEdit = (EditText) findViewById(R.id.ROS_NODE_IP_EDIT);
        if(!isNewRosNodeIP) {
            isNewRosNodeIP = true;
            rosNodeIPSpinner.setVisibility(View.GONE);
            enterNewRosNodeIPBtn.setText("Back to list");
            enterNewRosNodeIPEdit.setVisibility(View.VISIBLE);
        }
        else{
            isNewRosNodeIP = false;
            rosNodeIPSpinner.setVisibility(View.VISIBLE);
            enterNewRosNodeIPBtn.setText("Set New IP");
            enterNewRosNodeIPEdit.setVisibility(View.GONE);
        }
    }

    public void toggleRosPrefix(View view) {
        rosPrefixSpinner = (Spinner) findViewById(R.id.ROS_PREFIX_SPINNER);
        enterNewRosPrefixBtn = (Button) findViewById(R.id.TOGGLE_ROS_PREFIX_BTN);
        enterNewRosPrefixEdit = (EditText) findViewById(R.id.ROS_PREFIX_EDIT);
        if(!isNewRosPrefix) {
            isNewRosPrefix = true;
            rosPrefixSpinner.setVisibility(View.GONE);
            enterNewRosPrefixBtn.setText("Back to list");
            enterNewRosPrefixEdit.setVisibility(View.VISIBLE);
        }
        else{
            isNewRosPrefix = false;
            rosPrefixSpinner.setVisibility(View.VISIBLE);
            enterNewRosPrefixBtn.setText("Set New Prefix");
            enterNewRosPrefixEdit.setVisibility(View.GONE);
        }
    }
    public void toggleNewNamespace(View view) {
        namespaceSpinner = (Spinner) findViewById(R.id.NAMESPACE_SPINNER);
        enterNewNamespaceBtn = (Button) findViewById(R.id.TOGGLE_NAMESPACE_BTN);
        enterNewNamespaceEdit = (EditText) findViewById(R.id.NAMESPACE_EDIT);
        if(!isNewNamespace) {
            isNewNamespace = true;
            namespaceSpinner.setVisibility(View.GONE);
            enterNewNamespaceBtn.setText("Back to list");
            enterNewNamespaceEdit.setVisibility(View.VISIBLE);
        }
        else{
            isNewNamespace = false;
            namespaceSpinner.setVisibility(View.VISIBLE);
            enterNewNamespaceBtn.setText("Set New Namespace");
            enterNewNamespaceEdit.setVisibility(View.GONE);
        }
    }
    */
/*
    public List<String> readFile(String fileName){
        List<String> str = new ArrayList<String>();
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
        }
        return str;
    }

    public List<String> reverseStr(List<String> str){
        List<String> revStr = new ArrayList<String>();
        int index;
        //Reverse str so that the most recent IP is on top of spinner
        for(index = str.size() - 1; index >= 0; index--){
            revStr.add(str.get(index));
        }
        return revStr;
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

            }
        }
        */
}