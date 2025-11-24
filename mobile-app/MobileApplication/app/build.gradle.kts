plugins {
    alias(libs.plugins.android.application)
    alias(libs.plugins.kotlin.android)
    alias(libs.plugins.kotlin.compose)
    id("jacoco")
}

android {
    namespace = "com.example.testapp"
    compileSdk {
        version = release(36)
    }

    defaultConfig {
        applicationId = "com.example.testapp"
        minSdk = 24
        targetSdk = 36
        versionCode = 1
        versionName = "1.0"

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }
    kotlinOptions {
        jvmTarget = "11"
    }
    buildFeatures {
        compose = true
    }
}

jacoco {
    toolVersion = "0.8.10"
}

dependencies {
    implementation(libs.androidx.core.ktx)
    implementation(libs.androidx.lifecycle.runtime.ktx)
    implementation(libs.androidx.activity.compose)
    implementation(platform(libs.androidx.compose.bom))
    implementation(libs.androidx.compose.ui)
    implementation(libs.androidx.compose.ui.graphics)
    implementation(libs.androidx.compose.ui.tooling.preview)
    implementation(libs.androidx.compose.material3)
    implementation(libs.androidx.compose.material3.adaptive.navigation.suite)
    implementation(libs.androidx.navigation.compose)
    testImplementation("junit:junit:4.13.2")
    androidTestImplementation(libs.androidx.junit)
    androidTestImplementation(libs.androidx.espresso.core)
    androidTestImplementation(platform(libs.androidx.compose.bom))
    androidTestImplementation(libs.androidx.compose.ui.test.junit4)
    debugImplementation(libs.androidx.compose.ui.tooling)
    debugImplementation(libs.androidx.compose.ui.test.manifest)
}

//task to generate coverage for debug unit tests
tasks.register(
    "jacocoTestReport",
    org.gradle.testing.jacoco.tasks.JacocoReport::class
) {
    //run debug unit tests first
    dependsOn("testDebugUnitTest")

    reports {
        xml.required.set(true)
        html.required.set(true)
    }

    val excludes = listOf(
        "**/R.class",
        "**/R$*.class",
        "**/*\$Companion.class",
        "**/BuildConfig.*",
        "**/Manifest*.*",
        "android/**/*.*"
    )

    //where debug classes typically end up for Android app modules
    classDirectories.setFrom(
        fileTree("${buildDir}/intermediates/javac/debug/classes") {
            exclude(excludes)
        }
    )

    // Kotlin + Java sources
    sourceDirectories.setFrom(files("src/main/java", "src/main/kotlin"))

    // JaCoCo execution data from debug unit tests
    executionData.setFrom(files("${buildDir}/jacoco/testDebugUnitTest.exec"))
}