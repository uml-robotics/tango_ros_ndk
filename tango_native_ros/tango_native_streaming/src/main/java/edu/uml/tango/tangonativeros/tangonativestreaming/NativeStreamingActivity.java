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

/**
 * The main activity of the hello depth perception example application.
 *
 * This activity shows a message and logs the depth information to the logcat. It is responsible for
 * hooking to the android lifecycle events to native code code which calls into Tango C API.
 */
public class NativeStreamingActivity extends Activity {

    public static String ros_master_jstr = "http://10.0.7.172:11311", ros_ip_jstr = "10.0.7.220", tango_prefix_jstr = "tango_brain_0/", namespace_jstr = "tango_brain_0";

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
            ros_master_jstr = savedInstanceState.getString("ROS_MASTER");
            ros_ip_jstr = savedInstanceState.getString("ROS_IP");
            tango_prefix_jstr = savedInstanceState.getString("TANGO_PREFIX");
            namespace_jstr = savedInstanceState.getString("NAMESPACE");
        } else {
            Intent intent = getIntent();
            ros_master_jstr = intent.getStringExtra("ROS_MASTER");
            ros_ip_jstr = intent.getStringExtra("ROS_IP");
            tango_prefix_jstr = intent.getStringExtra("TANGO_PREFIX");
            namespace_jstr = intent.getStringExtra("NAMESPACE");
        }
        setContentView(R.layout.activity_depth_perception);
        TangoJniNative.onCreate(this);
    }

    @Override
    protected void onResume() {
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

    @Override
    protected void onPause() {
        TangoJniNative.onPause();
        unbindService(mTangoServiceConnection);
        super.onPause();
    }

    @Override
    public void onSaveInstanceState(Bundle savedInstanceState) {
        savedInstanceState.putString("ROS_MASTER", ros_master_jstr);
        savedInstanceState.putString("ROS_IP", ros_ip_jstr);
        savedInstanceState.putString("TANGO_PREFIX", tango_prefix_jstr);
        savedInstanceState.putString("NAMESPACE", namespace_jstr);

        super.onSaveInstanceState(savedInstanceState);
    }

    @Override
    protected void onRestoreInstanceState(Bundle savedInstanceState) {
        super.onRestoreInstanceState(savedInstanceState);
        ros_master_jstr = savedInstanceState.getString("ROS_MASTER");
        ros_ip_jstr = savedInstanceState.getString("ROS_IP");
        tango_prefix_jstr = savedInstanceState.getString("TANGO_PREFIX");
        namespace_jstr = savedInstanceState.getString("NAMESPACE");
    }
}
