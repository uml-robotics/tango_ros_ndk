/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*
 *Modifications:
 *
 * Copyright (c) 2016, University Of Massachusetts Lowell
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University of Massachusetts Lowell nor the names
 * from of its contributors may be used to endorse or promote products
 * derived this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * Author: Carlos Ibarra <clopez@cs.uml.edu>
*/
package edu.uml.tango.tangonativeros.tangonativestreaming;

import android.app.Activity;
import android.content.ComponentName;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.Bundle;
import android.os.IBinder;
import android.util.Log;
import android.view.View;

// TODO find a less hacky way to force the ros node to restart on app lifecycle transitions, currently feels like I am torturing the app lifecycle by stopping and starting the activity manually, maybe by only stopping the node if the ip of the master has changed
/**
 * The main activity of the hello depth perception example application.
 *
 * This activity shows a message and logs the depth information to the logcat. It is responsible for
 * hooking to the android lifecycle events to native code code which calls into Tango C API.
 */
public class NativeStreamingActivity extends Activity {

    public static String master_prefix, ros_master, master_port, ros_ip, tango_prefix, namespace,
                    ros_master_jstr, ros_ip_jstr, tango_prefix_jstr, namespace_jstr;

    public boolean isPaused = false, needsRestart = false;

    // Tango Service connection.
    ServiceConnection mTangoServiceConnection = new ServiceConnection() {
        public void onServiceConnected(ComponentName name, IBinder service) {
            TangoJniNative.onTangoServiceConnected(service);
        }

        public void onServiceDisconnected(ComponentName name) {
            // Handle this if you need to gracefully shutdown/retry
            // in the event that Tango itself crashes/gets upgraded while running.
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        if (savedInstanceState != null) {
            ros_master_jstr = savedInstanceState.getString("ROS_MASTER_JSTR");
            ros_ip_jstr = savedInstanceState.getString("ROS_IP_JSTR");
            tango_prefix_jstr = savedInstanceState.getString("TANGO_PREFIX_JSTR");
            namespace_jstr = savedInstanceState.getString("NAMESPACE_JSTR");
            master_prefix = savedInstanceState.getString("MASTER_PREFIX");
            ros_master = savedInstanceState.getString("ROS_MASTER");
            master_port = savedInstanceState.getString("MASTER_PORT");
            ros_ip = savedInstanceState.getString("ROS_IP");
            tango_prefix = savedInstanceState.getString("TANGO_PREFIX");
            namespace = savedInstanceState.getString("NAMESPACE");
        } else {
            Intent intent = getIntent();
            master_prefix = intent.getStringExtra("MASTER_PREFIX");
            ros_master = intent.getStringExtra("ROS_MASTER");
            master_port = intent.getStringExtra("MASTER_PORT");
            ros_ip = intent.getStringExtra("ROS_IP");
            tango_prefix = intent.getStringExtra("TANGO_PREFIX");
            namespace = intent.getStringExtra("NAMESPACE");
        }

        setContentView(R.layout.activity_depth_perception);
        TangoJniNative.onCreate(this);
    }

    boolean hasConfigChanged() {
        boolean restart = false;
        if (ros_master_jstr != null || ros_ip_jstr != null || tango_prefix_jstr != null || namespace_jstr != null) {
            restart = (ros_master_jstr != master_prefix + ros_master + ':' + master_port) || (ros_ip_jstr != ros_ip) || (tango_prefix_jstr != tango_prefix) || (namespace_jstr != namespace);
        }
        return restart;
    }

    @Override
    protected void onRestart() {
        super.onRestart();
//        Intent intent = new Intent(this, NativeStreamingActivity.class);
//        intent.putExtra("MASTER_PREFIX", master_prefix);
//        intent.putExtra("ROS_MASTER", ros_master);
//        intent.putExtra("MASTER_PORT", master_port);
//        intent.putExtra("ROS_IP", ros_ip);
//        intent.putExtra("TANGO_PREFIX", tango_prefix);
//        intent.putExtra("NAMESPACE", namespace);
//        startActivity(intent);
//        finish();
//        System.exit(0);
    }

    @Override
    protected void onNewIntent(Intent intent) {
        super.onNewIntent(intent);
        Log.d("new intent", "gotten");

    }

    @Override
    protected void onResume() {

        needsRestart = hasConfigChanged();

        ros_master_jstr = master_prefix + ros_master + ':' + master_port;
        ros_ip_jstr = ros_ip;
        tango_prefix_jstr = tango_prefix;
        namespace_jstr = namespace;

        if (needsRestart) {
            needsRestart = false;
//            Intent intent = new Intent(this, NativeStreamingActivity.class);
//            intent.putExtra("MASTER_PREFIX", master_prefix);
//            intent.putExtra("ROS_MASTER", ros_master);
//            intent.putExtra("MASTER_PORT", master_port);
//            intent.putExtra("ROS_IP", ros_ip);
//            intent.putExtra("TANGO_PREFIX", tango_prefix);
//            intent.putExtra("NAMESPACE", namespace);
            Intent intent = getIntent();
            startActivity(intent);
            finish();
            System.exit(0);
        } else {

            super.onResume();

            Intent intent = new Intent();
            intent.setClassName("com.google.tango", "com.google.atap.tango.TangoService");
            boolean success = (getPackageManager().resolveService(intent, 0) != null);
            // Attempt old service name
            if (!success) {
                intent = new Intent();
                intent.setClassName("com.projecttango.tango", "com.google.atap.tango.TangoService");
            }
            bindService(intent, mTangoServiceConnection, BIND_AUTO_CREATE);
            TangoJniNative.onResume(this);
        }
    }

    @Override
    protected void onPause() {
        isPaused = isFinishing();
        TangoJniNative.onPause();
        unbindService(mTangoServiceConnection);
        super.onPause();
//        Intent i = getBaseContext().getPackageManager()
//                .getLaunchIntentForPackage( getBaseContext().getPackageName() );
//        i.putExtras(getIntent());
//        i.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP);
//        startActivity(i);
    }

    @Override
    public void onSaveInstanceState(Bundle savedInstanceState) {
        savedInstanceState.putBoolean("IS_PAUSED", isPaused);
        savedInstanceState.putString("ROS_MASTER_JSTR", ros_master_jstr);
        savedInstanceState.putString("ROS_IP_JSTR", ros_ip_jstr);
        savedInstanceState.putString("TANGO_PREFIX_JSTR", tango_prefix_jstr);
        savedInstanceState.putString("NAMESPACE_JSTR", namespace_jstr);
        savedInstanceState.putString("MASTER_PREFIX", master_prefix);
        savedInstanceState.putString("ROS_MASTER", ros_master);
        savedInstanceState.putString("MASTER_PORT", master_port);
        savedInstanceState.putString("ROS_IP", ros_ip);
        savedInstanceState.putString("TANGO_PREFIX", tango_prefix);
        savedInstanceState.putString("NAMESPACE", namespace);
        super.onSaveInstanceState(savedInstanceState);
    }

    @Override
    protected void onRestoreInstanceState(Bundle savedInstanceState) {
        super.onRestoreInstanceState(savedInstanceState);
        isPaused = savedInstanceState.getBoolean("IS_PAUSED");
        ros_master_jstr = savedInstanceState.getString("ROS_MASTER_JSTR");
        ros_ip_jstr = savedInstanceState.getString("ROS_IP_JSTR");
        tango_prefix_jstr = savedInstanceState.getString("TANGO_PREFIX_JSTR");
        namespace_jstr = savedInstanceState.getString("NAMESPACE_JSTR");
        master_prefix = savedInstanceState.getString("MASTER_PREFIX");
        ros_master = savedInstanceState.getString("ROS_MASTER");
        master_port = savedInstanceState.getString("MASTER_PORT");
        ros_ip = savedInstanceState.getString("ROS_IP");
        tango_prefix = savedInstanceState.getString("TANGO_PREFIX");
        namespace = savedInstanceState.getString("NAMESPACE");
    }

    @Override
    public void onBackPressed() {
        stopStreaming(null);
    }

    public void stopStreaming(View view) {
//        onPause();
        Intent i = getBaseContext().getPackageManager()
                .getLaunchIntentForPackage( getBaseContext().getPackageName() );
        i.putExtras(getIntent());
        i.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP);
        startActivity(i);
        finish();
//        System.exit(0);
    }
}
