package edu.uml.tango.tangonativeros.tangonativestreaming;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.TextView;

import java.net.InetAddress;
import java.net.NetworkInterface;
import java.util.Collections;
import java.util.List;


public class SettingsActivity extends Activity {

    public static String ros_master = "http://10.0.4.20:11311",
                         ros_ip = "",
                         tango_prefix = "tango_brain_0/",
                         namespace = "tango_brain_0";
    public TextView ros_master_edit, tango_addr_edit, prefix_edit, tango_namespace_edit,
                    err_no_master;

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


        ros_ip = getIPAddress(true);
        setContentView(R.layout.activity_settings);
        ros_master_edit = (TextView) findViewById(R.id.MASTER_IP_EDIT);
        tango_addr_edit = (TextView) findViewById(R.id.NODE_IP_EDIT);
        prefix_edit = (TextView) findViewById(R.id.PREFIX_EDIT);
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
        ros_master = ros_master_edit.getText().toString();
        ros_ip = tango_addr_edit.getText().toString();
        tango_prefix = prefix_edit.getText().toString();
        namespace = tango_namespace_edit.getText().toString();
        Log.d("ROS Master URI: ", ros_master);
        Log.d("Tango IP: ", ros_ip);
        Log.d("Tango prefix: ", tango_prefix);
        Log.d("Tango Namespace: ", namespace);
        Intent intent = new Intent(this, NativeStreamingActivity.class);
        intent.putExtra("ROS_MASTER", ros_master);
        intent.putExtra("ROS_IP", ros_ip);
        intent.putExtra("TANGO_PREFIX", tango_prefix);
        intent.putExtra("NAMESPACE", namespace);
        intent.addFlags(Intent.FLAG_ACTIVITY_NO_HISTORY);
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
        tango_addr_edit = (TextView) findViewById(R.id.NODE_IP_EDIT);
        prefix_edit = (TextView) findViewById(R.id.PREFIX_EDIT);
        tango_namespace_edit = (TextView) findViewById(R.id.NAMESPACE_EDIT);
        ros_master_edit.setText(ros_master);
        tango_addr_edit.setText(ros_ip);
        prefix_edit.setText(tango_prefix);
        tango_namespace_edit.setText(namespace);
    }

}
