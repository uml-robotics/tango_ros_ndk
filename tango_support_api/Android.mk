#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# 

# Modifications:
# 
#  Copyright (c) 2016, University Of Massachusetts Lowell
#  All rights reserved.
# 
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
# 
#  1. Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#  notice, this list of conditions and the following disclaimer in the
#  documentation and/or other materials provided with the distribution.
#  3. Neither the name of the University of Massachusetts Lowell nor the names
#  from of its contributors may be used to endorse or promote products
#  derived this software without specific prior written permission.
# 
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#  Author: Carlos Ibarra <clopez@cs.uml.edu>
# 

LOCAL_PATH := $(call my-dir)
PROJECT_ROOT:= $(LOCAL_PATH)/..

include $(CLEAR_VARS)
LOCAL_MODULE := tango_support_api
ifeq ($(TARGET_ARCH),x86)
    LOCAL_SRC_FILES := $(PROJECT_ROOT)/EXTRACT_TANGO_SUPPORT_LIBRARY_HERE/libtango_support_api/x86/libtango_support_api.so
    LOCAL_EXPORT_C_INCLUDES := $(PROJECT_ROOT)/EXTRACT_TANGO_SUPPORT_LIBRARY_HERE/libtango_support_api/x86
else
    LOCAL_SRC_FILES := $(PROJECT_ROOT)/EXTRACT_TANGO_SUPPORT_LIBRARY_HERE/libtango_support_api/armeabi-v7a/libtango_support_api.so
    LOCAL_EXPORT_C_INCLUDES := $(PROJECT_ROOT)/EXTRACT_TANGO_SUPPORT_LIBRARY_HERE/libtango_support_api/armeabi-v7a
endif
include $(PREBUILT_SHARED_LIBRARY)

$(call import-add-path,$(PROJECT_ROOT))