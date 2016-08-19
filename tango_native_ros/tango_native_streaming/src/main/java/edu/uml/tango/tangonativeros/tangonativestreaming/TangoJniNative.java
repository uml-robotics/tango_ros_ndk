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
import android.os.IBinder;
import android.util.Log;

import java.io.File;

/**
 * Interfaces between C and Java.
 *
 * Note that these are the functions that call into native code, native code is
 * responsible for the communication between the application and Tango Service.
 */
public class TangoJniNative {

    /*
     *Code From TangoInitializationHelper, under License:
     * Copyright 2016 Google Inc. All Rights Reserved.
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

    public static final int ARCH_ERROR = -2;
    public static final int ARCH_FALLBACK = -1;
    public static final int ARCH_DEFAULT = 0;
    public static final int ARCH_ARM64 = 1;
    public static final int ARCH_ARM32 = 2;
    public static final int ARCH_X86_64 = 3;
    public static final int ARCH_X86 = 4;

    public static final int loadTangoSharedLibrary() {
        int loadedSoId = ARCH_ERROR;
        String basePath = "/data/data/com.google.tango/libfiles/";
        if (!(new File(basePath).exists())) {
            basePath = "/data/data/com.projecttango.tango/libfiles/";
        }
        Log.i("TangoJNINative", "basePath: " + basePath);

        try {
            System.load(basePath + "arm64-v8a/libtango_client_api.so");
            loadedSoId = ARCH_ARM64;
            Log.i("TangoJNINative", "Success! Using arm64-v8a/libtango_client_api.");
        } catch (UnsatisfiedLinkError e) {
        }
        if (loadedSoId < ARCH_DEFAULT) {
            try {
                System.load(basePath + "armeabi-v7a/libtango_client_api.so");
                loadedSoId = ARCH_ARM32;
                Log.i("TangoJNINative", "Success! Using armeabi-v7a/libtango_client_api.");
            } catch (UnsatisfiedLinkError e) {
            }
        }
        if (loadedSoId < ARCH_DEFAULT) {
            try {
                System.load(basePath + "x86_64/libtango_client_api.so");
                loadedSoId = ARCH_X86_64;
                Log.i("TangoJNINative", "Success! Using x86_64/libtango_client_api.");
            } catch (UnsatisfiedLinkError e) {
            }
        }
        if (loadedSoId < ARCH_DEFAULT) {
            try {
                System.load(basePath + "x86/libtango_client_api.so");
                loadedSoId = ARCH_X86;
                Log.i("TangoJNINative", "Success! Using x86/libtango_client_api.");
            } catch (UnsatisfiedLinkError e) {
            }
        }
        if (loadedSoId < ARCH_DEFAULT) {
            try {
                System.load(basePath + "default/libtango_client_api.so");
                loadedSoId = ARCH_DEFAULT;
                Log.i("TangoJNINative", "Success! Using default/libtango_client_api.");
            } catch (UnsatisfiedLinkError e) {
            }
        }
        if (loadedSoId < ARCH_DEFAULT) {
            try {
                System.loadLibrary("tango_client_api");
                loadedSoId = ARCH_FALLBACK;
                Log.i("TangoJNINative", "Falling back to libtango_client_api.so symlink.");
            } catch (UnsatisfiedLinkError e) {
            }
        }
        return loadedSoId;
    }
    //Code from InitializationHelper Ends here

    static {
        // This project depends on tango_client_api, so we need to make sure we load
        // the correct library first.
        if (loadTangoSharedLibrary() == ARCH_ERROR) {
            Log.e("TangoJNINative", "ERROR! Unable to load libtango_client_api.so!");
        }
        System.loadLibrary("tango_native_streaming");
    }

    /**
     * Interfaces to native OnCreate function.
     *
     * @param callerActivity the caller activity of this function.
     */
    public static native void onCreate(Activity callerActivity);

    /*
     * Called when the Tango service is connected.
     *
     * @param binder The native binder object.
     */
    public static native void onTangoServiceConnected(IBinder binder);

    /**
     * Interfaces to native OnPause function.
     */
    public static native void onPause();
}
