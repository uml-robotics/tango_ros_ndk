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
import android.widget.TextView;


/**
 * The main activity of the hello depth perception example application.
 *
 * This activity shows a message and logs the depth information to the logcat. It is responsible for
 * hooking to the android lifecycle events to native code code which calls into Tango C API.
 */
public class NativeStreamingActivity extends Activity {

    public static String ros_master, ros_ip, tango_prefix, namespace,
                    ros_master_jstr, ros_ip_jstr, tango_prefix_jstr, namespace_jstr;

    public boolean isPaused = false,
                   needsRestart = false,
                   nativeError = false,
                   tango_service_bound = false;

    // Tango Service connection.
    ServiceConnection mTangoServiceConnection;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        if (savedInstanceState != null) {
            ros_master_jstr = savedInstanceState.getString("ROS_MASTER_JSTR");
            ros_ip_jstr = savedInstanceState.getString("ROS_IP_JSTR");
            tango_prefix_jstr = savedInstanceState.getString("TANGO_PREFIX_JSTR");
            namespace_jstr = savedInstanceState.getString("NAMESPACE_JSTR");
            ros_master = savedInstanceState.getString("ROS_MASTER");
            ros_ip = savedInstanceState.getString("ROS_IP");
            tango_prefix = savedInstanceState.getString("TANGO_PREFIX");
            namespace = savedInstanceState.getString("NAMESPACE");
            isPaused = savedInstanceState.getBoolean("IS_PAUSED");
        } else {
            Intent intent = getIntent();
            ros_master = intent.getStringExtra("ROS_MASTER");
            ros_ip = intent.getStringExtra("ROS_IP");
            tango_prefix = intent.getStringExtra("TANGO_PREFIX");
            namespace = intent.getStringExtra("NAMESPACE");
            isPaused = intent.getBooleanExtra("IS_PAUSED", false);
        }

        setContentView(R.layout.activity_depth_perception);
        TangoJniNative.onCreate(this);
    }

    boolean hasConfigChanged() {
        boolean restart = false;
        restart |= ros_master_jstr == null;
        restart |= ros_ip_jstr == null;
        restart |= tango_prefix_jstr == null;
        restart |= namespace_jstr == null;
        if (ros_master_jstr != null && ros_master != null) restart |= !ros_master_jstr.equals(ros_master);
        if (ros_ip_jstr != null && ros_ip != null) restart |= !ros_ip_jstr.equals(ros_ip);
        if (tango_prefix_jstr != null && tango_prefix != null) restart |= !tango_prefix_jstr.equals(tango_prefix);
        if (namespace_jstr != null && namespace != null) restart |= !namespace_jstr.equals(namespace);
        return restart;
    }

    @Override
    protected void onRestart() {
        super.onRestart();
    }

    @Override
    protected void onNewIntent(Intent intent) {
        super.onNewIntent(intent);
        Log.d("new intent", "gotten");

    }

    @Override
    protected void onResume() {
        super.onResume();
        if (!isPaused) {

            mTangoServiceConnection = null;

            mTangoServiceConnection = new ServiceConnection() {

                public void onServiceConnected(ComponentName name, IBinder service) {
                    TangoJniNative.onTangoServiceConnected(service);
                    tango_service_bound = true;
                    Log.i("Tango Service bound", "");
                }

                public void onServiceDisconnected(ComponentName name) {
                    // Handle this if you need to gracefully shutdown/retry
                    // in the event that Tango itself crashes/gets upgraded while running.
                    tango_service_bound = false;
                    Log.i("Tango Service unbound","");
                }


            };

            needsRestart = hasConfigChanged();

            ros_master_jstr = ros_master;
            ros_ip_jstr = ros_ip;
            tango_prefix_jstr = tango_prefix;
            namespace_jstr = namespace;

            if (needsRestart) {
                needsRestart = false;
                Intent intent = getIntent();
                startActivity(intent);
                finish();
                //System.exit(0);
                Log.i("How did I get here?", "");
            } else {

                Intent intent = new Intent();
                intent.setClassName("com.google.tango", "com.google.atap.tango.TangoService");
                boolean success = (getPackageManager().resolveService(intent, 0) != null);
                // Attempt old service name
                if (!success) {
                    intent = new Intent();
                    intent.setClassName("com.projecttango.tango", "com.google.atap.tango.TangoService");
                }
                getApplicationContext().bindService(intent, mTangoServiceConnection, BIND_AUTO_CREATE);
                TangoJniNative.onResume(this);

                if (nativeError) {
                    Log.i("Native Error", "");
                    nativeError = false;
                    intent = getIntent();
                    intent.putExtra("IS_PAUSED", true);
                    startActivity(intent);
                    finish();
                    System.exit(0);
                }
            }
        } else {
            TextView msg = (TextView) findViewById(R.id.STREAM_STATE_LBL);
            msg.setText(R.string.STREAM_ERR);
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        isPaused = nativeError;
        TangoJniNative.onPause();
        try {
            if (tango_service_bound && mTangoServiceConnection != null) {
                getApplicationContext().unbindService(mTangoServiceConnection);
                mTangoServiceConnection = null;
            }
        } catch (Throwable e) {
            Log.e(e.toString(), "");
        }
    }

    @Override
    public void onSaveInstanceState(Bundle savedInstanceState) {
        savedInstanceState.putBoolean("IS_PAUSED", isPaused);
        savedInstanceState.putString("ROS_MASTER_JSTR", ros_master_jstr);
        savedInstanceState.putString("ROS_IP_JSTR", ros_ip_jstr);
        savedInstanceState.putString("TANGO_PREFIX_JSTR", tango_prefix_jstr);
        savedInstanceState.putString("NAMESPACE_JSTR", namespace_jstr);
        savedInstanceState.putString("ROS_MASTER", ros_master);
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
        ros_master = savedInstanceState.getString("ROS_MASTER");
        ros_ip = savedInstanceState.getString("ROS_IP");
        tango_prefix = savedInstanceState.getString("TANGO_PREFIX");
        namespace = savedInstanceState.getString("NAMESPACE");
    }

    @Override
    public void onBackPressed() {
        stopStreaming(null);
    }

    public void stopStreaming(View view) {
        Intent i = getBaseContext().getPackageManager()
                .getLaunchIntentForPackage( getBaseContext().getPackageName() );
        i.putExtras(getIntent());
        i.putExtra("IS_PAUSED", false);
        //i.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP);
        startActivity(i);
        finish();
        System.exit(0);
    }
}
