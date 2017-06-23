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
    private Button backToListMasterPrefixBtn;
    private Button enterNewMasterPrefixBtn;

    private boolean isNewMasterIP = false;
    private Spinner masterIPSpinner;
    private EditText enterNewMasterIPEdit;
    private Button backToListMasterIPBtn;
    private Button enterNewMasterIPBtn;
    private List<String> str = new ArrayList<String>();
    private List<String> revStr = new ArrayList<String>();

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

        masterPrefixSpinner = (Spinner) findViewById(R.id.MASTER_PREFIX_SPINNER);
        ArrayAdapter<String> adapter = new ArrayAdapter<String>(this, android.R.layout.simple_spinner_item, revStr);
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        masterPrefixSpinner.setAdapter(adapter);
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
//button leads to here
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

    public void enterNewMasterPrefix(View view) {
        isNewMasterPrefix = true;
        masterPrefixSpinner = (Spinner) findViewById(R.id.MASTER_PREFIX_SPINNER);
        masterPrefixSpinner.setVisibility(View.GONE);
        enterNewMasterPrefixBtn = (Button) findViewById(R.id.NEW_MASTER_PREFIX_BTN);
        enterNewMasterPrefixBtn.setVisibility(View.GONE);
        enterNewMasterPrefixEdit = (EditText) findViewById(R.id.MASTER_PREFIX_EDIT);
        enterNewMasterPrefixEdit.setVisibility(View.VISIBLE);
        backToListMasterPrefixBtn = (Button) findViewById(R.id.BACK_TO_LIST_MASTER_PREFIX_BTN);
        backToListMasterPrefixBtn.setVisibility(View.VISIBLE);
    }

    public void backToListMasterPrefix(View view) {
        isNewMasterPrefix = false;
        enterNewMasterPrefixEdit = (EditText) findViewById(R.id.MASTER_PREFIX_EDIT);
        enterNewMasterPrefixEdit.setVisibility(View.GONE);
        backToListMasterPrefixBtn = (Button) findViewById(R.id.BACK_TO_LIST_MASTER_PREFIX_BTN);
        backToListMasterPrefixBtn.setVisibility(View.GONE);
        masterPrefixSpinner = (Spinner) findViewById(R.id.MASTER_PREFIX_SPINNER);
        masterPrefixSpinner.setVisibility(View.VISIBLE);
        enterNewMasterPrefixBtn = (Button) findViewById(R.id.NEW_MASTER_PREFIX_BTN);
        enterNewMasterPrefixBtn.setVisibility(View.VISIBLE);
    }

    public void enterNewMasterIP(View view) {
        isNewMasterIP = true;
        masterIPSpinner = (Spinner) findViewById(R.id.MASTER_IP_SPINNER);
        masterIPSpinner.setVisibility(View.GONE);
        enterNewMasterIPBtn = (Button) findViewById(R.id.NEW_MASTER_IP_BTN);
        enterNewMasterIPBtn.setVisibility(View.GONE);
        enterNewMasterIPEdit = (EditText) findViewById(R.id.MASTER_IP_EDIT);
        enterNewMasterIPEdit.setVisibility(View.VISIBLE);
        backToListMasterIPBtn = (Button) findViewById(R.id.BACK_TO_LIST_MASTER_IP_BTN);
        backToListMasterIPBtn.setVisibility(View.VISIBLE);
    }

    public void backToListMasterIP(View view) {
        isNewMasterIP = false;
        enterNewMasterIPEdit = (EditText) findViewById(R.id.MASTER_IP_EDIT);
        enterNewMasterIPEdit.setVisibility(View.GONE);
        backToListMasterIPBtn = (Button) findViewById(R.id.BACK_TO_LIST_MASTER_IP_BTN);
        backToListMasterIPBtn.setVisibility(View.GONE);
        masterIPSpinner = (Spinner) findViewById(R.id.MASTER_IP_SPINNER);
        masterIPSpinner.setVisibility(View.VISIBLE);
        enterNewMasterIPBtn = (Button) findViewById(R.id.NEW_MASTER_IP_BTN);
        enterNewMasterIPBtn.setVisibility(View.VISIBLE);
    }

}
