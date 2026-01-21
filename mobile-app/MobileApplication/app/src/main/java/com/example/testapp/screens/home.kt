package com.example.testapp.screens

import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import kotlinx.coroutines.flow.StateFlow

import androidx.compose.material3.Button
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Arrangement

@Composable
fun home(
    logs: StateFlow<List<String>>,
    status: StateFlow<String>
) {
    val logList by logs.collectAsState()
    val connectionStatus by status.collectAsState()

    Column(modifier = Modifier.fillMaxSize().padding(16.dp)) {
        Text(
            text = "Status: $connectionStatus",
            style = MaterialTheme.typography.headlineMedium
        )
        Text(
            text = "BLE Logs:",
            style = MaterialTheme.typography.titleMedium,
            modifier = Modifier.padding(top = 16.dp, bottom = 8.dp)
        )
        LazyColumn(modifier = Modifier.weight(1f)) {
            items(logList) { log ->
                Text(
                    text = log,
                    style = MaterialTheme.typography.bodySmall
                )
            }
        }
    }
}
