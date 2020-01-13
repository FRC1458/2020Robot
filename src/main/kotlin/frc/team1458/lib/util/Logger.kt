package frc.team1458.lib.util

import frc.team1458.lib.util.flow.systemTimeMillis
import kotlinx.coroutines.*
import java.io.*


class Logger(private val logFolderPath: String = "/logs/", private val writeDelayMs: Long = 10) {
    private val logList: ArrayList<Pair<String, () -> String>> = ArrayList()
    private lateinit var logFile: FileWriter
    private lateinit var logWriter: BufferedWriter
    private lateinit var loggerJob: Job
    private var running: Boolean = false

    init {
        try {
            logFile = FileWriter(getLogFilePath())
            logWriter = BufferedWriter(logFile)
        } catch (e: Exception) {
            println("ERROR: INIT " + stackTraceToString(e))
        }
    }

    private fun getLogFilePath(): String {
        return try {
            val tempFile: String = logFolderPath + "logNumber.txt"
            val file = File(tempFile)

            if (!file.isFile && !file.createNewFile()) {
                throw IOException("Error creating new file: " + file.absolutePath)
            }

            val tempLogReader = FileReader(tempFile)
            var logNumber = 0

            logNumber = try {
                tempLogReader.readText().trim().toInt() + 1
            } catch (e: Exception) {
                0
            }
            tempLogReader.close()

            val tempLogWriter = FileWriter(tempFile)
            tempLogWriter.write(logNumber.toString())
            tempLogWriter.close()

            logFolderPath + "Log$logNumber.csv"
        } catch (e: Exception) {
            println("ERROR: LOGPATH " + stackTraceToString(e))
            "/tmp/LogERROR.csv"
        }
    }

    private fun setup() {
        try {
            logFile = FileWriter(getLogFilePath())
            logWriter = BufferedWriter(logFile)

            var header: String = "ts"

            for (keyPair: Pair<String, () -> String> in logList) {
                header+=("," + keyPair.first)
            }

            header+="\n"
            println(header)

            logWriter.write(header)
            logWriter.flush()
        } catch (e: Exception) {
            println("ERROR: SETUP " + stackTraceToString(e))
        }
    }

    fun addParameter(parameterName: String, getData: () -> String) {
        logList.add(Pair(parameterName, getData))
    }

    private fun log() {
        try {
            var toWrite: String = "$systemTimeMillis"

            for (keyPair: Pair<String, () -> String> in logList) {
                toWrite += ("," + keyPair.second())
            }

            toWrite += "\n"

            logWriter.write(toWrite)
            logWriter.flush()
        }
        catch (e: Exception) {
            println("ERROR: LOG " + stackTraceToString(e))
        }
    }

    fun start() {
        setup()

        running = true

        loggerJob = GlobalScope.launch {
            while (true) {
                log()
                delay(writeDelayMs)
                if (!running) {
                    break
                }
            }
        }

        loggerJob.start()
    }

    fun stop() {
        running = false
    }

    suspend fun blockingStop() {
        if (loggerJob.isActive) {
            loggerJob.cancelAndJoin()
        }
    }

    fun clearLogDirectory() {
        try {
            val dir = File(logFolderPath)
            for (file in dir.listFiles()!!) if (!file.isDirectory) file.delete()
        }
        catch (e: Exception) {
            println("ERROR: CLEAR LOG DIR " + stackTraceToString(e))
        }
    }

    // TODO Move this out
    private fun stackTraceToString(e: Throwable): String? {
        val sb = StringBuilder()
        for (element in e.stackTrace) {
            sb.append(element.toString())
            sb.append("\n")
        }
        return sb.toString()
    }
}