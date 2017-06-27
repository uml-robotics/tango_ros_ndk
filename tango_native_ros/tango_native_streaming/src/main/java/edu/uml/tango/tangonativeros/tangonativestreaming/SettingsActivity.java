package edu.uml.tango.tangonativeros.tangonativestreaming;

import android.app.Activity;
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
import java.io.InputStreamReader;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class SettingsActivity extends Activity {

    public static String master_prefix = "http://",
                         ros_master = "10.0.7.172",
                         master_port = "11311",
                         ros_ip = "",
                         tango_prefix = "tango_brain_0/",
                         namespace = "tango_brain_0";
    public TextView ros_master_prefix_edit, ros_master_edit, ros_port_edit, tango_addr_edit, prefix_edit, tango_namespace_edit;
    private boolean isNewMasterPrefix = false;
    private Spinner masterPrefixSpinner;
    private EditText enterNewMasterPrefixEdit;
    private Button enterNewMasterPrefixBtn;

    private boolean isNewMasterIP = false;
    private Spinner masterIPSpinner;
    private EditText enterNewMasterIPEdit;
    private Button enterNewMasterIPBtn;

    private boolean isNewPort = false;
    private Spinner portSpinner;
    private EditText enterNewPortEdit;
    private Button enterNewPortBtn;

    private boolean isNewNodeIP = false;
    private Spinner nodeIPSpinner;
    private EditText enterNewNodeIPEdit;
    private Button enterNewNodeIPBtn;

    private boolean isNewRosPrefix = false;
    private Spinner rosPrefixSpinner;
    private EditText enterNewRosPrefixEdit;
    private Button enterNewRosPrefixBtn;

    private boolean isNewNamespace = false;
    private Spinner namespaceSpinner;
    private EditText enterNewNamespaceEdit;
    private Button enterNewNamespaceBtn;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        Intent intent = getIntent();
        if (intent.hasExtra("MASTER_PREFIX"))
            master_prefix = intent.getStringExtra("MASTER_PREFIX");
        if (intent.hasExtra("ROS_MASTER"))
            ros_master = intent.getStringExtra("ROS_MASTER");
        if (intent.hasExtra("MASTER_PORT"))
            master_port = intent.getStringExtra("MASTER_PORT");
        if (intent.hasExtra("ROS_IP"))
            ros_ip = intent.getStringExtra("ROS_IP");
        if (intent.hasExtra("TANGO_PREFIX"))
            tango_prefix = intent.getStringExtra("TANGO_PREFIX");
        if (intent.hasExtra("NAMESPACE"))
            namespace = intent.getStringExtra("NAMESPACE");


        ros_ip = getIPAddress(true);
        setContentView(R.layout.activity_settings);
        ros_master_prefix_edit = (TextView) findViewById(R.id.MASTER_PREFIX_EDIT);
        ros_master_edit = (TextView) findViewById(R.id.MASTER_IP_EDIT);
        ros_port_edit = (TextView) findViewById(R.id.PORT_EDIT);
        tango_addr_edit = (TextView) findViewById(R.id.NODE_IP_EDIT);
        prefix_edit = (TextView) findViewById(R.id.ROS_PREFIX_EDIT);
        tango_namespace_edit = (TextView) findViewById(R.id.NAMESPACE_EDIT);

        if (savedInstanceState != null) {
            master_prefix = savedInstanceState.getString("MASTER_PREFIX");
            ros_master = savedInstanceState.getString("ROS_MASTER");
            master_port = savedInstanceState.getString("MASTER_PORT");
            ros_ip = savedInstanceState.getString("ROS_IP");
            tango_prefix = savedInstanceState.getString("TANGO_PREFIX");
            namespace = savedInstanceState.getString("NAMESPACE");
        }
        ros_master_prefix_edit.setText(master_prefix);
        ros_master_edit.setText(ros_master);
        ros_port_edit.setText(master_port);

        if (!ros_ip.equals("")) {
            tango_addr_edit.setText(ros_ip);
        } else {
            tango_addr_edit.setText("tango_addr_error");
        }

        prefix_edit.setText(tango_prefix);

        tango_namespace_edit.setText(namespace);

        List<String> masterPrefixData = readFile("previousDataMasterPrefix");
        List<String> masterIPData = readFile("previousDataMasterIP");
        List<String> portData = readFile("previousDataPort");
        List<String> rosIPData = readFile("previousDataRosIP");
        List<String> rosPrefixData = readFile("previousDataRosPrefix");
        List<String> namespaceData = readFile("previousDataNamespace");

        masterPrefixSpinner = (Spinner) findViewById(R.id.MASTER_PREFIX_SPINNER);
        ArrayAdapter<String> adapterMasterPrefix = new ArrayAdapter<String>(this, android.R.layout.simple_spinner_item, masterPrefixData);
        adapterMasterPrefix.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        masterPrefixSpinner.setAdapter(adapterMasterPrefix);

        masterIPSpinner = (Spinner) findViewById(R.id.MASTER_IP_SPINNER);
        ArrayAdapter<String> adapterMasterIP = new ArrayAdapter<String>(this, android.R.layout.simple_spinner_item, masterIPData);
        adapterMasterIP.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        masterIPSpinner.setAdapter(adapterMasterIP);

        portSpinner = (Spinner) findViewById(R.id.PORT_SPINNER);
        ArrayAdapter<String> adapterPort = new ArrayAdapter<String>(this, android.R.layout.simple_spinner_item, portData);
        adapterPort.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        portSpinner.setAdapter(adapterPort);

        nodeIPSpinner = (Spinner) findViewById(R.id.NODE_IP_SPINNER);
        ArrayAdapter<String> adapterNodeIP = new ArrayAdapter<String>(this, android.R.layout.simple_spinner_item, rosIPData);
        adapterNodeIP.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        nodeIPSpinner.setAdapter(adapterNodeIP);

        rosPrefixSpinner = (Spinner) findViewById(R.id.ROS_PREFIX_SPINNER);
        ArrayAdapter<String> adapterRosPrefix = new ArrayAdapter<String>(this, android.R.layout.simple_spinner_item, rosPrefixData);
        adapterRosPrefix.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        rosPrefixSpinner.setAdapter(adapterRosPrefix);

        namespaceSpinner = (Spinner) findViewById(R.id.NAMESPACE_SPINNER);
        ArrayAdapter<String> adapterNamespace = new ArrayAdapter<String>(this, android.R.layout.simple_spinner_item, namespaceData);
        adapterNamespace.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        namespaceSpinner.setAdapter(adapterNamespace);
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
        master_prefix = ros_master_prefix_edit.getText().toString();
        ros_master = ros_master_edit.getText().toString();
        master_port = ros_port_edit.getText().toString();
        ros_ip = tango_addr_edit.getText().toString();
        tango_prefix = prefix_edit.getText().toString();
        namespace = tango_namespace_edit.getText().toString();
        Log.d("ROS Master prefix", master_prefix);
        Log.d("ROS Master IP: ", ros_master);
        Log.d("ROS port: ", master_port);
        Log.d("Tango IP: ", ros_ip);
        Log.d("Tango prefix: ", tango_prefix);
        Log.d("Tango Namespace: ", namespace);
        Intent intent = new Intent(this, NativeStreamingActivity.class);
        intent.putExtra("MASTER_PREFIX", master_prefix);
        intent.putExtra("ROS_MASTER", ros_master);
        intent.putExtra("MASTER_PORT", master_port);
        intent.putExtra("ROS_IP", ros_ip);
        intent.putExtra("TANGO_PREFIX", tango_prefix);
        intent.putExtra("NAMESPACE", namespace);
        startActivity(intent);
        //finish();
    }

    @Override
    public void onSaveInstanceState(Bundle savedInstanceState) {
        super.onSaveInstanceState(savedInstanceState);
        master_prefix = ros_master_prefix_edit.getText().toString();
        ros_master = ros_master_edit.getText().toString();
        master_port = ros_port_edit.getText().toString();
        ros_ip = tango_addr_edit.getText().toString();
        tango_prefix = prefix_edit.getText().toString();
        namespace = tango_namespace_edit.getText().toString();
        savedInstanceState.putString("MASTER_PREFIX", master_prefix);
        savedInstanceState.putString("ROS_MASTER", ros_master);
        savedInstanceState.putString("MASTER_PORT", master_port);
        savedInstanceState.putString("ROS_IP", ros_ip);
        savedInstanceState.putString("TANGO_PREFIX", tango_prefix);
        savedInstanceState.putString("NAMESPACE", namespace);
    }

    @Override
    public void onRestoreInstanceState(Bundle savedInstanceState) {
        super.onRestoreInstanceState(savedInstanceState);
        master_prefix = savedInstanceState.getString("MASTER_PREFIX");
        ros_master = savedInstanceState.getString("ROS_MASTER");
        master_port = savedInstanceState.getString("MASTER_PORT");
        ros_ip = savedInstanceState.getString("ROS_IP");
        tango_prefix = savedInstanceState.getString("TANGO_PREFIX");
        namespace = savedInstanceState.getString("NAMESPACE");
        ros_master_prefix_edit = (TextView) findViewById(R.id.MASTER_PREFIX_EDIT);
        ros_master_edit = (TextView) findViewById(R.id.MASTER_IP_EDIT);
        ros_port_edit = (TextView) findViewById(R.id.PORT_EDIT);
        tango_addr_edit = (TextView) findViewById(R.id.NODE_IP_EDIT);
        prefix_edit = (TextView) findViewById(R.id.ROS_PREFIX_EDIT);
        tango_namespace_edit = (TextView) findViewById(R.id.NAMESPACE_EDIT);
        ros_master_prefix_edit.setText(master_prefix);
        ros_master_edit.setText(ros_master);
        ros_port_edit.setText(master_port);
        tango_addr_edit.setText(ros_ip);
        prefix_edit.setText(tango_prefix);
        tango_namespace_edit.setText(namespace);
    }
//TODO: Use "STRING_NAME" + "STRING_TYPE" for ids and prints
    public void toggleMasterPrefix(View view) {
        masterPrefixSpinner = (Spinner) findViewById(R.id.MASTER_PREFIX_SPINNER);
        enterNewMasterPrefixBtn = (Button) findViewById(R.id.TOGGLE_MASTER_PREFIX_BTN);
        enterNewMasterPrefixEdit = (EditText) findViewById(R.id.MASTER_PREFIX_EDIT);
        if(!isNewMasterPrefix) {
            isNewMasterPrefix = true;
            masterPrefixSpinner.setVisibility(View.GONE);
            enterNewMasterPrefixBtn.setText("Back to list");
            enterNewMasterPrefixEdit.setVisibility(View.VISIBLE);
        }
        else {
            isNewMasterPrefix = false;
            masterPrefixSpinner.setVisibility(View.VISIBLE);
            enterNewMasterPrefixBtn.setText("Set New Prefix");
            enterNewMasterPrefixEdit.setVisibility(View.GONE);
        }
    }

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

    public void togglePort(View view) {
        portSpinner = (Spinner) findViewById(R.id.PORT_SPINNER);
        enterNewPortBtn = (Button) findViewById(R.id.TOGGLE_PORT_BTN);
        enterNewPortEdit = (EditText) findViewById(R.id.PORT_EDIT);
        if(!isNewPort) {
            isNewPort = true;
            portSpinner.setVisibility(View.GONE);
            enterNewPortBtn.setText("Back to list");
            enterNewPortEdit.setVisibility(View.VISIBLE);
        }
        else{
            isNewPort = false;
            portSpinner.setVisibility(View.VISIBLE);
            enterNewPortBtn.setText("Set New Port");
            enterNewPortEdit.setVisibility(View.GONE);
        }
    }

    public void toggleNodeIP(View view) {
        nodeIPSpinner = (Spinner) findViewById(R.id.NODE_IP_SPINNER);
        enterNewNodeIPBtn = (Button) findViewById(R.id.TOGGLE_NODE_IP_BTN);
        enterNewNodeIPEdit = (EditText) findViewById(R.id.NODE_IP_EDIT);
        if(!isNewNodeIP) {
            isNewNodeIP = true;
            nodeIPSpinner.setVisibility(View.GONE);
            enterNewNodeIPBtn.setText("Back to list");
            enterNewNodeIPEdit.setVisibility(View.VISIBLE);
        }
        else{
            isNewNodeIP = false;
            nodeIPSpinner.setVisibility(View.VISIBLE);
            enterNewNodeIPBtn.setText("Set New IP");
            enterNewNodeIPEdit.setVisibility(View.GONE);
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

    public List<String> readFile(String fileName){
        List<String> str = new ArrayList<String>();
        List<String> revStr = new ArrayList<String>();

        String line;
        int index;
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
        //Reverse str so that the most recent IP is on top
        for(index = str.size() - 1; index >= 0; index--){
            revStr.add(str.get(index));
        }
        return revStr;
    }
}
