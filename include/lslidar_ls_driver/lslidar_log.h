/******************************************************************************
 * This file is part of lslidar_cx driver.
 *
 * Copyright 2022 LeiShen Intelligent Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/


#define LS_ERROR   std::cout << "\033[1m\033[31m"  // bold red
#define LS_MSG     std::cout << "\033[1m\033[32m"  // bold green
#define LS_WARN    std::cout << "\033[1m\033[33m"  // bold yellow
#define LS_PARAM   std::cout << "\033[1m\033[34m"  // bold blue
#define LS_PCAP    std::cout << "\033[1m\033[35m"  // bold purple
#define LS_SOCKET  std::cout << "\033[1m\033[36m"  // bold cyan
#define LS_INFO    std::cout << "\033[1m\033[37m"  // bold white
#define LS_END     "\033[0m" << std::endl