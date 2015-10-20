#!/bin/bash

ssh root@192.168.8.2 "date -us @`date -u +%s`"
ssh -X debian@192.168.8.2

