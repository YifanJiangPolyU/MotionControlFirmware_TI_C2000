
uint32      Idx                    # obj index and subindex
uint8       Length
uint8       AccessType             # read or write
uint8       AccessResult           # access results
uint16[2]   Data                   # holds data to be writen to obj (in case of write)
                                   # or data returned from obj (in case of read)
