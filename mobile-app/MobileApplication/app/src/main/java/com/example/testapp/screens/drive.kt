package com.example.testapp.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.viewmodel.compose.viewModel
import com.example.testapp.ViewModel.ViewModel

@Composable
fun Drive(viewModel: ViewModel) {
    val status1 = viewModel.status1.collectAsState()
    val status2 = viewModel.status2.collectAsState()
    val status3 = viewModel.status3.collectAsState()
    val status4 = viewModel.status4.collectAsState()

    Column(Modifier.padding(8.dp)) {
        Text("drive page")

        StatusRow(
            s1 = status1.value,
            s2 = status2.value,
            s3 = status3.value,
            s4 = status4.value
        )
    }
}

@Composable
fun StatusBadge(isGood: Boolean) {
    val backgroundColor = if (isGood) Color(0xFF4CAF50) else Color(0xFFF44336)

    Box(
        modifier = Modifier
            .size(16.dp)
            .background(backgroundColor, CircleShape)
    )
}

@Composable
fun StatusItem(
    label: String,
    isGood: Boolean
) {
    Row(
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.spacedBy(3.dp)
    ) {
        StatusBadge(isGood = isGood)
        Text(
            text = label,
            fontSize = 12.sp
        )
    }
}

@Composable
fun StatusRow(
    s1: Boolean,
    s2: Boolean,
    s3: Boolean,
    s4: Boolean
) {
    Row(
        modifier = Modifier.padding(16.dp),
        horizontalArrangement = Arrangement.spacedBy(8.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        StatusItem("Rear Camera", s1)
        StatusItem("Side Radar", s2)
        StatusItem("Blindspot", s3)
        StatusItem("Front Camera", s4)
    }
}
