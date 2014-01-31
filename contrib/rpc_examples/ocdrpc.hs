-- OpenOCD RPC example, covered by GNU GPLv3 or later
-- Copyright (C) 2014 Paul Fertser
--
-- Example output:
-- $ ./ocdrpc
-- Halting the target, full log output captured:
-- target state: halted
-- target halted due to debug-request, current mode: Thread
-- xPSR: 0x21000000 pc: 0x00003352 msp: 0x20000fe8
--
-- Read memory, parse the result and show as a list of strings:
-- ["0x20001000","0x0000334d","0x00002abb","0x0000118f","0x00002707","0x00002707","0x00002707","0x00000000","0x00000000","0x00000000","0x00000000","0x00002707","0x00002707","0x00000000","0x00002707","0x00002781"]
-- Resuming

{-# LANGUAGE OverloadedStrings #-}
module Main where

import Prelude
import Control.Applicative
import Network.Socket
import System.IO.Streams.Core hiding (connect)
import System.IO.Streams.Network
import System.IO.Streams.Attoparsec
import Data.Attoparsec.ByteString.Char8
import Data.Attoparsec.Combinator
import Data.ByteString.Char8 hiding (putStrLn, concat, map)
import Text.Printf

ocdReply = manyTill anyChar (char '\x1a')

ocdExec (oistream, oostream) command = do
  write (Just $ pack $ command ++ "\x1a") oostream
  parseFromStream ocdReply oistream

-- For each line: dispose of address, then match hex values
mdwParser = (manyTill anyChar (string ": ") *>
              hexadecimal `sepBy` char ' ')
            `sepBy` string " \n"

ocdMdw :: (InputStream ByteString, OutputStream ByteString) -> Integer -> Integer -> IO [Integer]
ocdMdw s start count = do
  s <- ocdExec s $ "ocd_mdw " ++ show start ++ " " ++ show count
  case parseOnly mdwParser (pack s) of
    Right r -> return $ concat r

main = do
  osock <- socket AF_INET Stream defaultProtocol
  haddr <- inet_addr "127.0.0.1"
  connect osock (SockAddrInet 6666 haddr)
  ostreams <- socketToStreams osock
  putStrLn "Halting the target, full log output captured:"
  ocdExec ostreams "capture \"halt\"" >>= putStrLn
  putStrLn "Read memory, parse the result and show as a list of strings:"
  ocdMdw ostreams 0 16 >>= putStrLn . (show :: [String] -> String) . map (printf "0x%08x")
  putStrLn "Resuming"
  ocdExec ostreams "resume"
