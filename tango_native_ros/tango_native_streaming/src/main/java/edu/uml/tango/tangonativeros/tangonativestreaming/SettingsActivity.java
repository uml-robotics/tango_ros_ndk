package edu.uml.tango.tangonativeros.tangonativestreaming;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.TextView;

import java.net.InetAddress;
import java.net.NetworkInterface;
import java.util.Collections;
import java.util.List;


public class SettingsActivity extends Activity {

    public static String ros_master = "10.0.7.172", master_prefix = "http://", master_port = ":11311", ros_ip = "", tango_prefix = "tango_brain_0/", namespace = "tango_brain_0";
    public TextView ros_addr, tango_addr, prefix, tango_namespace;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        ros_ip = getIPAddress(true);

        setContentView(R.layout.activity_settings);

        ros_addr = (TextView) findViewById(R.id.MASTER_IP_EDIT);
        tango_addr = (TextView) findViewById(R.id.NODE_IP_EDIT);
        prefix = (TextView) findViewById(R.id.PREFIX_EDIT);
        tango_namespace = (TextView) findViewById(R.id.NAMESPACE_EDIT);

        ros_addr.setText(ros_master);

        if (!ros_ip.equals("")) {
            tango_addr.setText(ros_ip);
        } else {
            tango_addr.setText("tango_addr_error");
        }

        prefix.setText(tango_prefix);

        tango_namespace.setText(namespace);
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
        ros_master = ros_addr.getText().toString();
        ros_ip = tango_addr.getText().toString();
        tango_prefix = prefix.getText().toString();
        namespace = tango_namespace.getText().toString();
        Intent intent = new Intent(this, NativeStreamingActivity.class);
        intent.putExtra("ROS_MASTER", master_prefix + ros_master + master_port);
        intent.putExtra("ROS_IP", ros_ip);
        intent.putExtra("TANGO_PREFIX", tango_prefix);
        intent.putExtra("NAMESPACE", namespace);
        startActivity(intent);
    }

}
