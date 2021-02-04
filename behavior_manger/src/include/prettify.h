/*!*********************************************************************************
 *  \file       prettify.h
 *  \brief      Prettify definition file.
 *  \details    This file contains the Prettify declaration.
 *              To obtain more information about it's definition consult
 *              the prettify.cpp file.
 *  \authors    Alberto Camporredondo.
 *  \copyright  Copyright (c) 2019 Universidad Politecnica de Madrid
 *              All rights reserved
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef _PRETTIFY_H
#define _PRETTIFY_H

/*GNU/Linux*/
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

/*Color definitions*/
#define ERROR "\033[1;31m"
#define OK "\033[1;32m"
#define INFO "\033[1;97m"
#define STOP "\033[0m"
#define WARNING "\033[1;33m"

/*Class definition*/
class Prettify
{
public: /*Constructor & Destructor*/
  Prettify();
  ~Prettify();

private: /*Internal functions*/
  std::string getColor(std::string operation);

public: /*Process functions*/
  /*Functionality*/
  void printParent(std::string operation, std::string message);
  void printChild(std::string operation, std::string message);
  void printChildList(std::string operation, std::string message, std::vector<std::string> elements);
  void printGrandChild(std::string operation, std::string message);
  void printGrandChildList(std::string operation, std::string message, std::vector<std::string> elements);
  void printError(std::string message);
  void printEnd(std::string message);
  void printTitle();
  void printWarningVector(std::string message, std::vector<std::string>);
};

#endif
