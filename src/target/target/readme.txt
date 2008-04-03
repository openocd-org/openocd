Prerequisites:
The users of OpenOCD as well as computer programs interacting with OpenOCD are expecting that certain commands 
do the same thing across all the targets.

Rules to follow when writing scripts:

1. The configuration script should be defined such as , for example, the following sequences are working:
	reset
	flash info <bank>
and
	reset 
	flash erase_address <start> <len>
	
In most cases this can be accomplished by specifying the default startup mode as reset_init (target command 
in the configuration file).
 
2. If the target is correctly configured, flash must be writable without any other helper commands. It is 
assumed that all write-protect mechanisms should be disabled.
