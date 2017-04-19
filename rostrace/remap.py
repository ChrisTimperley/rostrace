#!/usr/bin/env python
from rosgraph.masterapi import Master

def main():
    m = Master('/remap')
    url = m.lookupService('hello_world')[2]
    print(url)

if __name__ == "__main__":
    main()
