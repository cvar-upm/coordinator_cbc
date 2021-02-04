/*!*******************************************************************************************
 *  \file       prettify.cpp
 *  \brief      Prettify implementation file.
 *  \details    This file implements the Prettify class.
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

#include "../include/prettify.h"

Prettify::Prettify() {}

Prettify::~Prettify() {}

void Prettify::printParent(std::string operation, std::string message)
{
  std::cout << std::endl;
  std::cout << getColor(operation) << "=> " << message << STOP << std::endl;
}

void Prettify::printChild(std::string operation, std::string message)
{
  std::cout << STOP << "\t|" << std::endl;
  std::cout << "\t---> " << getColor(operation) << message << STOP << std::endl;
}

void Prettify::printChildList(std::string operation, std::string message, std::vector<std::string> elements)
{
  std::cout << STOP << "\t|" << std::endl;
  std::cout << "\t---> " << getColor(operation) << message << ": [ ";
  int i = 0, j = 0;
  std::string array_elements = "", line = "";
  for (auto element : elements)
  {
    line += element;
    array_elements += element;
    i++;
    j++;
    if (i < elements.size())
    {
      array_elements += ", ";
      if (line.size() >= 20)
      {
        array_elements += "\n\t\t\t\t  ";
        line = "";
        j = 0;
      }
    }
  }
  std::cout << array_elements << " ]" << STOP << std::endl;
}

void Prettify::printGrandChild(std::string operation, std::string message)
{
  std::cout << STOP << "\t\t|" << std::endl;
  std::cout << "\t\t---> " << getColor(operation) << message << STOP << std::endl;
}

void Prettify::printGrandChildList(std::string operation, std::string message, std::vector<std::string> elements)
{
  std::cout << STOP << "\t\t\t|" << std::endl;
  std::cout << "\t\t\t---> " << getColor(operation) << message << ": [ ";
  int i = 0, j = 0;
  std::string array_elements = "", line = "";
  for (auto element : elements)
  {
    line += element;
    array_elements += element;
    i++;
    j++;
    if (i < elements.size())
    {
      array_elements += ", ";
      if (line.size() >= 20)
      {
        array_elements += "\n\t\t\t\t\t       º";
        line = "";
        j = 0;
      }
    }
  }
  std::cout << array_elements << " ]" << STOP << std::endl;
}

void Prettify::printError(std::string message)
{
  std::cout << std::endl;
  std::cout << ERROR << "[ERROR]: " << message << STOP << std::endl;
}

void Prettify::printWarningVector(std::string message, std::vector<std::string> vector)
{
  std::cout << std::endl;
  std::cout << WARNING << "[WARNING]: " << message;
  for (int i = 0; i < vector.size(); i++)
  {
    if (vector.size() - 1 == i)
    {
      std::cout << vector[i];
    }
    else
    {
      std::cout << vector[i] << ", ";
    }
  }
  std::cout << STOP << std::endl;
}

void Prettify::printEnd(std::string message)
{
  std::cout << std::endl;
  std::cout << OK << "[DONE]: " << message << STOP << std::endl;
  std::cout << std::endl;
}

std::string Prettify::getColor(std::string operation)
{
  std::transform(operation.begin(), operation.end(), operation.begin(), ::toupper);
  if (operation == "INFO")
    return /*INFO*/ "";
  if (operation == "ERROR")
    return ERROR;
  if (operation == "OK")
    return OK;
  if (operation == "WARNING")
    return WARNING;
  else
    return STOP;
}

void Prettify::printTitle()
{
  std::cout << std::endl;
  std::cout << OK << "\n= = = > Behavior Catalog < = = = " << STOP << std::endl;
  std::cout << std::endl;
}
